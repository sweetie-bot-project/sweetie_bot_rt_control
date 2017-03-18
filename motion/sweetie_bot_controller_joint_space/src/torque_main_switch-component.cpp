#include "torque_main_switch-component.hpp"

#include <rtt/Component.hpp>

#include "sweetie_bot_orocos_misc/get_subservice_by_type.hpp"

using namespace RTT;
using namespace std;

namespace sweetie_bot {
namespace motion {
namespace controller {


std::ostream& operator<<(std::ostream& s, const std::vector<std::string>& strings) 
{
	s << "[ ";
	for(auto it = strings.begin(); it != strings.end(); it++) s << *it << ", ";
	s << " ]";
	return s;
}

TorqueMainSwitch::TorqueMainSwitch(std::string const& name)  : 
	TaskContext(name, RTT::base::TaskCore::PreOperational),
	log(logger::categoryFromComponentName(name))
{
	this->provides()->doc("In operatioanl state switch servos torque off and feedfodward actual position to reference.");

	// ports
	// PORTS: input
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event triggers controller execution cycle.");

	this->addPort("in_joints_actual", in_joints_port).
		doc("Full sorted actual robot pose (from sensors or from agregator).");

	// PORTS: output
	this->addPort("out_joints_ref", out_joints_port).
		doc("Reference joint positions for agregator.");

	// PROPERTIES
	this->addProperty("herkulex_scheds", herkulex_scheds)
		.doc("List of HerkulexSched components should be stopped when servos are being switched off.");

	this->addProperty("herkulex_arrays", herkulex_arrays)
		.doc("List of controlled HerkulexArrays components.");

	this->addProperty("velocity_zeroing", velocity_zeroing)
		.doc("Set velocity to zero before coping it to reference pose. Can be handy if velocity measurements are noisy.")
		.set(false);

	// operations: provided
	this->addOperation("rosSetOperational", &TorqueMainSwitch::rosSetOperational, this)
		.doc("ROS compatible start/stop operation (std_srvs::SetBool).");

	// Service: reqires
	robot_model = new sweetie_bot::motion::RobotModel(this);
	this->requires()->addServiceRequester(ServiceRequester::shared_ptr(robot_model));
	
	// other actions
	log(INFO) << "TorqueMainSwitch is constructed!" << endlog();
}

bool TorqueMainSwitch::configureHook()
{
	// INITIALIZATION
	// check if ResourceClient Service presents
	resource_client = getSubServiceByType<ResourceClientInterface>(this->provides().get());
	if (!resource_client) {
		log(ERROR) << "ResourceClient plugin is not loaded." << endlog();
		return false;
	}
	// resource_client->setResourceChangeHook(boost::bind(&TorqueMainSwitch::resourceChangeHook, this));
	// check if RobotModel Service presents
	if (!robot_model->ready()) {
		log(ERROR) << "RobotModel service is not ready." << endlog();
		return false;
	}
	//int n_joints_fullpose = robot_model->listJoints("").size();
	// allocate memory
	in_joints_port.getDataSample(actual_fullpose);
	// set ports data samples
	out_joints_port.setDataSample(actual_fullpose);

	// Get necessary OperationCallers
	if (herkulex_arrays.size() == 0) {
		log(WARN) << "No HerkulexArrays is provided." << endlog();
		return false;
	}
	if (herkulex_arrays.size() != herkulex_scheds.size()) {
		log(WARN) << "Number of HerkulexArrays and HerkulexScheds is not same. Configuration may be wrong." << endlog();
	}
	// TODO multiple component support
	//if (herkulex_arrays.size() > 1) log(WARN) << "Mutiple HerkulexArray support is not implemented yet." << endlog();

	// check HerkulexArrays
	// form joint list and get OperationCallers
	std::vector<std::string> joints;
	torque_off_callers.clear();
	for(auto array_name = herkulex_arrays.begin(); array_name != herkulex_arrays.end(); array_name++) {
		// get HerkulexArray 
		TaskContext * herk_array = this->getPeer(*array_name);
		if ( !herk_array ) {
			log(ERROR) << "Peer with name `" << *array_name << "` is not found." << endlog();
			return false;
		}
		// get setAllServosTorqueFree operation
		torque_off_callers.push_back( OperationCaller< bool(bool) >(herk_array->getOperation("setAllServosTorqueFree"), this->engine()) );
		if ( ! torque_off_callers.back().ready() ) {
			log(ERROR) << "`" << *array_name << ".setAllServosTorqueFree()` operation is not ready." << endlog();
			return false;
		}
		// get list of servos
		OperationCaller< vector<string>() > list_joints_call(herk_array->getOperation("listServos"), this->engine()) ;
		if ( ! list_joints_call.ready() ) {
			log(ERROR) << "`" << *array_name << ".listServos` operation is not ready." << endlog();
			return false;
		}
		std::vector<std::string> new_joints = list_joints_call();
		joints.insert(joints.end(), new_joints.begin(), new_joints.end());
	}
	// form list of necessary resources
	controlled_chains = robot_model->getJointChains(joints);

	// check HerkulexSched
	for(auto sched_name = herkulex_scheds.begin(); sched_name != herkulex_scheds.end(); sched_name++) {
		if ( ! this->getPeer(*sched_name) ) {
			log(ERROR) << "Peer with name `" << *sched_name << "` is not found." << endlog();
			return false;
		}
	}
	
	if (log(DEBUG)) {
		log() << "Herkulex schedulers: [ "; 
		for(auto it = herkulex_scheds.begin(); it != herkulex_scheds.end(); it++) log() << *it << ", ";
		log() << " ]." << endlog();
		log() << "Herkulex arrays: [ "; 
		for(auto it = herkulex_arrays.begin(); it != herkulex_arrays.end(); it++) log() << *it << ", ";
		log() << " ]." << endlog();
		log() << "Controlled chains: [ "; 
		for(auto it = controlled_chains.begin(); it != controlled_chains.end(); it++) log() << *it << ", ";
		log() << " ]." << endlog();
	}

	log(INFO) << "TorqueMainSwitch is configured !" << endlog();
	return true;
}

/* 
 * Tries to make the controller operational. 
 *
 * Sends a resource request. Later a reply to which will be processed by the plugin's 
 * callback resourceChangedHook. If it returns true controller will become operational.
 * in the updateHook if all resources were allocated or not, if some are lacking.
 * This is checked using the controller's resourceChangedHook.
 */
bool TorqueMainSwitch::startHook()
{
	in_joints_port.getDataSample(actual_fullpose);
	// request resources
	resource_client->resourceChangeRequest(controlled_chains);
	// unconditionally switch servos off
	log(INFO) << "Setting servo torque OFF." << endlog();
	if ( !setAllServosTorqueFree(true) ) {
		log(ERROR) << "Unable to set servo torque!" << endlog();
		return false;
	}
	// clear sync port buffer
	RTT::os::Timer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// now update hook will be periodically executed
	log(INFO) << "TorqueMainSwitch is started !" << endlog();
	return true;
}

bool TorqueMainSwitch::setSchedulersActive(bool is_active) 
{
	bool success = true;

	for(auto sched_name = herkulex_scheds.begin(); sched_name != herkulex_scheds.end(); sched_name++) {
		TaskContext * sched = this->getPeer(*sched_name);
		if (!sched) {
			success = false;
			log(WARN) << "Unable to get scheduler: " << *sched_name << endlog();
			continue;
		}
		if (!is_active && sched->isRunning()) {
			if (!sched->stop()) {
				success = false;
				log(ERROR) << "Error during stopping scheduler " << *sched_name << endlog();
			}
		}
		else if(is_active && !sched->isRunning()) {
			if (!sched->start()) {
				success = false;
				log(ERROR) << "Error during starting  scheduler " << *sched_name << endlog();
			}
		}
		else {
			// no action performed: someone already stopped or started scheduler
			success = false;
		}
	}
	return success;
}

bool TorqueMainSwitch::setAllServosTorqueFree(bool torque_is_off) 
{
	bool success = true;
	// switch off all scedulers
	bool was_running = setSchedulersActive(false);

	// switch all servos torque off
	// TODO separate sructure for each array
	// TODO asyncronic call
	// TODO retry 
	for(auto it = torque_off_callers.begin(); it != torque_off_callers.end(); it++) {
		if ( !(it->ready() && it->call(torque_is_off)) ) {
			success = false;
			log(ERROR) << "Error during `setAllServoTorqueFree` call." << endlog();
		}
	}
	// switch on all scedulers
	setSchedulersActive(was_running);

	return success;
}

/*bool TorqueMainSwitch::resourceChangeHook() 
{
	// depending on resourse posession 
	// determine servos new state
	bool servos_troque_is_off = resource_client->hasResources(controlled_chains);

	// change servos state
	log(INFO) << "Setting servo torque " << (!servos_troque_is_off) ? "ON" : "OFF" << endlog();
	if (setAllServosTorqueFree(servos_torque_is_off)) {
		log(INFO) << "Servos torque is " << (!servos_troque_is_off) ? "ON" : "OFF" << endlog(); 
		// we are operational if servos are off	
		return servos_troque_is_off;
	}
	else {
		log(ERROR) << "Unable to set servo torque!" << endlog();
		return false;
	}
}*/

void TorqueMainSwitch::updateHook()
{
	// let resource_client do it stuff
	resource_client->step();	

	// syncronize with sync messages
	{
		RTT::os::Timer::TimerId unused;
		if (sync_port.read(unused) != NewData) return;
	}

	// main operational 
	int state = resource_client->getState();
	if (state & ResourceClient::OPERATIONAL) {
		// read port
		in_joints_port.read(actual_fullpose, true);
		// set velocity to zero if necessary
		if (velocity_zeroing) actual_fullpose.velocity.assign(actual_fullpose.velocity.size(), 0.0);
		// publish new reference position
		out_joints_port.write(actual_fullpose);
	}
	else if (state == ResourceClient::NONOPERATIONAL) {
		log(INFO) << "TorqueMainSwitch is exiting  operational state !" << endlog();
		this->stop();
	}
}

/* 
 * Preempts the controllers and releases its resources.
 */
void TorqueMainSwitch::stopHook() 
{
	// user calls stop() directly 
	if (resource_client->isOperational()) resource_client->stopOperational();
	// change servos state
	log(INFO) << "Setting servo torque ON" << endlog();
	if ( !setAllServosTorqueFree(false) ) {
		log(ERROR) << "Unable to set servo torque!" << endlog();
	}
	log(INFO) << "TorqueMainSwitch is stopped!" << endlog();
}

void TorqueMainSwitch::cleanupHook() 
{
	controlled_chains.clear();
	torque_off_callers.clear();
	log(INFO) << "TorqueMainSwitch cleaning up !" << endlog();
}

/**
 * ROS comaptible start/stop operation.
 */
bool TorqueMainSwitch::rosSetOperational(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
	if (req.data) {
		resp.success = start();
		resp.message = "start() is called.";
	}
	else {
		stop();
		resp.success = true;
		resp.message = "stop() is called.";
	}
	return true;
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(TorqueMainSwitch)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::motion::controller::TorqueMainSwitch)
