#include <sweetie_bot_orocos_misc/stream_operators.hpp>

#include "solver_ik_analytical.hpp"

#include <sweetie_bot_logger/logger.hpp>

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

class SolverIKLeg : public SolverIKAnalytical
{
public:
	SolverIKLeg(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance) : 
		SolverIKAnalytical(chain, lower, upper, tolerance)
	{}

	static std::unique_ptr<SolverIKAnalytical> createSolver(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log);
	bool solveIK_impl(const KDL::Frame& b_T_e, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const override;

private:
	static bool is_registered;
};

bool SolverIKLeg::is_registered = SolverIKFactoryAnalytical::Register(&SolverIKLeg::createSolver);

std::unique_ptr<SolverIKAnalytical> SolverIKLeg::createSolver(const KDL::Chain& chain, const KDL::JntArray& lower, const KDL::JntArray& upper, const KDL::Twist& tolerance, sweetie_bot::logger::Logger& log) 
{
	const double eps = 1e-6;
	KDL::Frame f_tip;
	const KDL::Joint * joint;
	KDL::Vector origin, axis;

	// check number of joints
	if (chain.getNrOfJoints() != 6 || chain.getNrOfSegments() != 6) {
		log(DEBUG) << "IK Leg: Number of joints and segments must be equal to 6."  << std::endl;
		return std::unique_ptr<SolverIKLeg>();
	}

	// check each segment
	for(int seg = 0; seg < 6; seg++) {
		// common references
		f_tip = chain.getSegment(seg).getFrameToTip();
		joint = &chain.getSegment(seg).getJoint();
		// joint specific limitations
		switch (seg) {
			case 0:
				origin = joint->JointOrigin();
				axis = Vector(joint->JointAxis().x(), 0.0, 0.0);
				break;
			case 1:
				origin = Vector::Zero();
				axis = Vector(0.0, joint->JointAxis().y(), 0.0);
				break;
			case 2:
				origin = Vector(0.0, 0.0, - std::abs(joint->JointOrigin().z()));
				axis = Vector(0.0, joint->JointAxis().y(), 0.0);
				break;
			case 3:
				origin = Vector(0.0, joint->JointOrigin().y(), - std::abs(joint->JointOrigin().z()));
				axis = Vector(0.0, joint->JointAxis().y(), 0.0);
				break;
			case 4:
				origin = Vector::Zero();
				axis = Vector(joint->JointAxis().x(), 0.0, 0.0);
				break;
			case 5:
				origin = Vector::Zero();
				axis = Vector(0.0, 0.0, joint->JointAxis().z());
				break;
		}
		// perform checks
		bool check_passed = Equal(f_tip, Frame(Rotation::Identity(), origin), eps) &&
							Equal(joint->JointOrigin(), origin, eps) || 
							Equal(joint->JointAxis(), axis, eps);
		if (!check_passed) {
			log(DEBUG) << "IK Leg: Kineamtic chain check: unsupported geometry  " << chain.getSegment(seg).getName() << " segment: origin " << joint->JointOrigin() << " axis " << joint->JointAxis() << " f_tip " << f_tip <<  std::endl;
			log(DEBUG) << "expected origin " << origin << " axis " << axis << " f_tip " << Frame(Rotation::Identity(), origin) << endlog();
			return std::unique_ptr<SolverIKAnalytical>();
		}
	}

	return std::unique_ptr<SolverIKLeg>(new SolverIKLeg(chain, lower, upper, tolerance));
}

bool SolverIKLeg::solveIK_impl(const KDL::Frame& b_T_e, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const
{
	const Joint& joint0 = chain_.getSegment(0).getJoint();
	const Joint& joint1 = chain_.getSegment(1).getJoint();
	const Joint& joint2 = chain_.getSegment(2).getJoint();
	const Joint& joint3 = chain_.getSegment(3).getJoint();
	const Joint& joint4 = chain_.getSegment(4).getJoint();
	const Joint& joint5 = chain_.getSegment(5).getJoint();

	// move to first joint origin
	KDL::Vector p = b_T_e.p - joint0.JointOrigin();
	log(DEBUG) << "IK solve: p = " << p << std::endl;

	//
	// calculate positions of JOINT0
	//
	jnt(0) = atan2(p.y(), -p.z());
	double shift_y_123 = joint3.JointOrigin().y();
	double pxy_norm = Vector2(p.y(), p.z()).Norm();
	// if target point is near axis of joint0 then precise analytical soltion may not exists
	// so use approximate solution on this area
	double d2;
	if (10.0*fabs(shift_y_123) < pxy_norm) {
		// precise solution: angle correction is less then 5 degrees
		jnt(0) -= asin( shift_y_123 / pxy_norm);
		d2 = p.x()*p.x() + p.y()*p.y() + p.z()*p.z()  - shift_y_123*shift_y_123;
	}
	else {
		// aproximate solution
		d2 = p.x()*p.x() + p.y()*p.y() + p.z()*p.z();
	}
	// reverse sign if necessaru
	jnt(0) *= joint0.JointAxis().x();
	log(DEBUG) << "IK leg: shift_y_123 = " << shift_y_123 << ", j0_uncorr = " << atan2(p.y(), -p.z())*joint0.JointAxis().x() << ", j0 = " << jnt(0) <<  std::endl;

	//
	// calculate positions of JOINT2
	//
	double l1 = - joint2.JointOrigin().z();
	double l2 = - joint3.JointOrigin().z();
	double cosJoint2 = - (d2 - l1*l1 - l2*l2) / (2.0*l1*l2);
	if (abs(cosJoint2) > 1.0001) {
		// out of rechability
		log(DEBUG) << "IK failed: out of rechability: d = " << sqrt(d2) << ", l1 = " << l1 << ", l2 = " << l2 << endlog();
		return false;
	}
	else if (cosJoint2 > 1.0) cosJoint2 = 1.0;
	else if (cosJoint2 < -1.0) cosJoint2 = -1.0;
	double theta2 = acos( cosJoint2 );
	const double joint2_sign = -1;
	jnt(2) = joint2_sign * (M_PI - theta2); 
	// reverse sign if necessaru
	jnt(2) *= joint2.JointAxis().y();
	log(DEBUG) << "IK leg:  d = " << sqrt(d2) << ", l1 = " << l1 << ", l2 = " << l2 << ", theta2 = " << theta2 << ", j2 = " << jnt(2) << std::endl;


	//
	// calculate positions of JOINT1
	//
	jnt(1) = - asin( p.x() / sqrt(d2) );
	jnt(1) -= joint2_sign * asin( l2 * sin(theta2) /  sqrt(d2) );
	// reverse sign if necessaru
	jnt(1) *= joint1.JointAxis().y();
	log(DEBUG) << "IK leg: theta1 = " << - asin( p.x() / sqrt(d2) ) << ", j1 = " << jnt(1) << std::endl;

	// 
	// calculate positions of JOINT3, JOINT4, JOINT5
	//
	KDL::Frame b_T_4s = joint0.pose(jnt(0)) * joint1.pose(jnt(1)) * joint2.pose(jnt(2)) * joint3.pose(0.0);
	b_T_4s.p -= joint0.JointOrigin();
	// check if result is sane
	/*if ( (b_T_4s.p - p).Norm() > tolerance_pos_ ) {
		log(DEBUG) << "IK leg: position tolerance exceeded: b_T3_s = " << b_T_4s << std::endl;
	}*/

	// transform axes: Y -> Z, X -> Y, Z -> X
	KDL::Rotation Rc = Rotation(0.0, 0.0, 1.0, 
	                      1.0, 0.0, 0.0, 
						  0.0, 1.0, 0.0);
	KDL::Rotation R = Rc*b_T_e.M.Inverse()*b_T_4s.M*Rc.Inverse();
	R.SetInverse();
	// calculate angles
	R.GetEulerZYX(jnt(3), jnt(4), jnt(5));
	// reverse signs if necessary
	jnt(3) *= joint3.JointAxis().y();
	jnt(4) *= joint4.JointAxis().x();
	jnt(5) *= joint5.JointAxis().z();

	// Debug output
	if (log(DEBUG)) {
		log() << "IK leg: b_T_3s = " << b_T_4s << " jnt345 = " << jnt.data.tail<3>().transpose() <<  std::endl;
		log() << "IK leg: b_R_3s = " << std::endl << b_T_4s.M.data[0] << b_T_4s.M.data[1] << b_T_4s.M.data[2] << std::endl << b_T_4s.M.data[3] << b_T_4s.M.data[4] << b_T_4s.M.data[5] << std::endl << b_T_4s.M.data[6] << b_T_4s.M.data[7] << b_T_4s.M.data[8] << std::endl;
		log() << "IK leg: R = " << std::endl << R.data[0] << R.data[1] << R.data[2] << std::endl << R.data[3] << R.data[4] << R.data[5] << std::endl << R.data[6] << R.data[7] << R.data[8] << std::endl;
		log() << endlog();
	}

	return true;
}

}
}

