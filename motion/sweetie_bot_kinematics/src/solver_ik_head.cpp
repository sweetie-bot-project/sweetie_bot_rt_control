#include <sweetie_bot_orocos_misc/stream_operators.hpp>

#include "solver_ik_analytical.hpp"

#include <sweetie_bot_logger/logger.hpp>

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

class SolverIKHead : public SolverIKAnalytical
{
public:
	SolverIKHead(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance) : 
		SolverIKAnalytical(chain, lower, upper, tolerance)
	{}

	static std::unique_ptr<SolverIKAnalytical> createSolver(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log);
	bool solveIK_impl(const KDL::Frame& b_T_e, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const override;

private:
	static bool is_registered;
}; 

bool SolverIKHead::is_registered = SolverIKFactoryAnalytical::Register(&SolverIKHead::createSolver);

std::unique_ptr<SolverIKAnalytical> SolverIKHead::createSolver(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) 
{
	const double eps = 1e-6;
	KDL::Frame f_tip;
	const KDL::Joint * joint;
	KDL::Vector origin, axis;

	// check number of joints
	if (chain.getNrOfJoints() != 4 || chain.getNrOfSegments() != 4) {
		log(DEBUG) << "IK Head: Number of joints and segments must be equal to 4."  << std::endl;
		return std::unique_ptr<SolverIKAnalytical>();
	}

	// check each segment
	for(int seg = 0; seg < 4; seg++) {
		// common references
		f_tip = chain.getSegment(seg).getFrameToTip();
		joint = &chain.getSegment(seg).getJoint();
		// joint specific limitations
		switch (seg) {
			case 0:
				origin = joint->JointOrigin();
				axis = Vector(0.0, joint->JointAxis().y(), 0.0);
				break;
			case 1:
				// ignore shift along 0x
				origin = Vector(0.0, joint->JointOrigin().y(), std::abs(joint->JointOrigin().z()));
				axis = Vector(0.0, 0.0, joint->JointAxis().z());
				break;
			case 2:
				origin = Vector::Zero();
				axis = Vector(0.0, joint->JointAxis().y(), 0.0);
				break;
			case 3:
				origin = Vector(0.0, 0.0, std::abs(joint->JointOrigin().z()));
				axis = Vector(joint->JointAxis().x(), 0.0, 0.0);
				break;
		}
		// perform checks
		bool check_passed = Equal(f_tip, Frame(Rotation::Identity(), origin), eps) &&
							Equal(joint->JointOrigin(), origin, eps) || 
							Equal(joint->JointAxis(), axis, eps);
		if (!check_passed) {
			log(DEBUG) << "IK Head: Kineamtic chain check: unsupported geometry  " << chain.getSegment(seg).getName() << " segment: origin " << joint->JointOrigin() << " axis " << joint->JointAxis() << " f_tip " << f_tip <<  std::endl;
			log(DEBUG) << "expected origin " << origin << " axis " << axis << " f_tip " << Frame(Rotation::Identity(), origin) << endlog();
			return std::unique_ptr<SolverIKAnalytical>();
		}
	}

	// return solver
	return std::unique_ptr<SolverIKAnalytical>(new SolverIKHead(chain, lower, upper, tolerance));
}

bool SolverIKHead::solveIK_impl(const KDL::Frame& b_T_e, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const
{
	const Joint& joint0 = chain_.getSegment(0).getJoint();
	const Joint& joint1 = chain_.getSegment(1).getJoint();
	const Joint& joint2 = chain_.getSegment(2).getJoint();
	const Joint& joint3 = chain_.getSegment(3).getJoint();

	// move to first joint origin
	KDL::Vector p = b_T_e.p - joint0.JointOrigin();
	log(DEBUG) << "IK head: p = " << p << std::endl;

	//
	// calculate positions of JOINT0
	//
	jnt(0) = atan2(p.x(), p.z());
	jnt(0) *= joint0.JointAxis().y();
	log(DEBUG) << "IK head: j0 = " << jnt(0) <<  std::endl;

	// 
	// calculate positions of JOINT1, JOINT2, JOINT3
	//
	Frame b_T_1s = joint0.pose(jnt(0)) * joint1.pose(0.0);
	Rotation R = b_T_1s.M.Inverse() * b_T_e.M;
	// calculate angles
	R.GetEulerZYX(jnt(1), jnt(2), jnt(3));
	// reverse signs if necessary
	jnt(1) *= joint1.JointAxis().z();
	jnt(2) *= joint2.JointAxis().y();
	jnt(3) *= joint3.JointAxis().x();

	// Debug output
	if (log(DEBUG)) {
		log() << "IK head: b_T_1s = " << b_T_1s << " jnt123 = " << jnt.data.tail<3>().transpose() <<  std::endl;
		log() << "IK head: b_R_1s = " << std::endl << b_T_1s.M.data[0] << b_T_1s.M.data[1] << b_T_1s.M.data[2] << std::endl << b_T_1s.M.data[3] << b_T_1s.M.data[4] << b_T_1s.M.data[5] << std::endl << b_T_1s.M.data[6] << b_T_1s.M.data[7] << b_T_1s.M.data[8] << std::endl;
		log() << "IK head: R = " << std::endl << R.data[0] << R.data[1] << R.data[2] << std::endl << R.data[3] << R.data[4] << R.data[5] << std::endl << R.data[6] << R.data[7] << R.data[8] << std::endl;
		log() << endlog();
	}

	return true;
}

}
}


