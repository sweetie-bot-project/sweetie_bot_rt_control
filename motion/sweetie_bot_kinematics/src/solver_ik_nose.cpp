#include "solver_ik.hpp"

#include <sweetie_bot_logger/logger.hpp>

#include "kdl_ostream.hpp"

using sweetie_bot::logger::Logger;
using namespace RTT;
using namespace KDL;

namespace sweetie_bot {
namespace motion {

class SolverIKNose : public SolverIK 
{
public:
	static std::unique_ptr<SolverIK> createSolver(const KDL::Chain& chain, sweetie_bot::logger::Logger& log);
	bool solveIK(const KDL::Chain& chain, const KDL::Frame& b_T_e, const KDL::JntArray& lower, const KDL::JntArray& upper, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const override;

private:
	static bool is_registered;
};

bool SolverIKNose::is_registered = SolverIKFactory::Register(&SolverIKNose::createSolver);

std::unique_ptr<SolverIK> SolverIKNose::createSolver(const KDL::Chain& chain, sweetie_bot::logger::Logger& log) 
{
	const double eps = 1e-6;
	KDL::Frame f_tip;
	const KDL::Joint * joint;
	KDL::Vector origin, axis;

	// check number of joints
	if (chain.getNrOfJoints() != 6 || chain.getNrOfSegments() != 6) {
		log(DEBUG) << "IK Nose: Number of joints and segments must be equal to 6."  << std::endl;
		return std::unique_ptr<SolverIK>();
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
			case 4:
				// ignore shift along 0x
				origin = Vector(std::abs(joint->JointOrigin().x()), 0.0, 0.0);
				axis = Vector(0.0, 0.0, joint->JointAxis().z());
				break;
			case 5:
				origin = Vector::Zero();
				axis = Vector(0.0, joint->JointAxis().y(), 0.0);
				break;
		}
		// perform checks
		bool check_passed = Equal(f_tip, Frame(Rotation::Identity(), origin), eps) &&
							Equal(joint->JointOrigin(), origin, eps) || 
							Equal(joint->JointAxis(), axis, eps);
		if (!check_passed) {
			log(DEBUG) << "IK Nose: Kineamtic chain check: unsupported geometry  " << chain.getSegment(seg).getName() << " segment: origin " << joint->JointOrigin() << " axis " << joint->JointAxis() << " f_tip " << f_tip <<  std::endl;
			log(DEBUG) << "expected origin " << origin << " axis " << axis << " f_tip " << Frame(Rotation::Identity(), origin) << endlog();
			return std::unique_ptr<SolverIK>();
		}
	}
	return std::unique_ptr<SolverIKNose>(new SolverIKNose());
}

bool SolverIKNose::solveIK(const KDL::Chain& chain, const KDL::Frame& b_T_e, const KDL::JntArray& lower, const KDL::JntArray& upper, KDL::JntArray& jnt, sweetie_bot::logger::Logger& log) const
{
	const Joint& joint0 = chain.getSegment(0).getJoint();
	const Joint& joint1 = chain.getSegment(1).getJoint();
	const Joint& joint2 = chain.getSegment(2).getJoint();
	const Joint& joint3 = chain.getSegment(3).getJoint();
	const Joint& joint4 = chain.getSegment(4).getJoint();
	const Joint& joint5 = chain.getSegment(5).getJoint();

	// move to first joint origin
	KDL::Vector p = b_T_e.p - joint0.JointOrigin();
	log(DEBUG) << "IK nose: p = " << p << std::endl;

	double h = joint3.JointOrigin().z();
	double l3 = joint4.JointOrigin().x();
	double l2 = sqrt(h*h + l3*l3);
	double l1 = joint1.JointOrigin().z();
	double theta2 = atan2(l3, h);

	//
	// calculate positions of JOINT0
	//
	double l2s2 = l2*l2 - p.y()*p.y();
	double pxz_norm2 = p.x()*p.x() + p.z()*p.z();
	double pxz_norm = sqrt(pxz_norm2);
	double cosTheta0 = - (l2s2 - pxz_norm2 - l1*l1) / (2.0*l1*pxz_norm);
	if (abs(cosTheta0) > 1.000) {
		// out of rechability
		log(DEBUG) << "IK nose failed: out of rechability: l1 = " << l1 << ", l2s = " << sqrt(l2s2) << ", Pxz_norm = " << pxz_norm << endlog();
		return false;
	}
	double theta0 = acos( cosTheta0 );
	double j0 = M_PI/2.0 - atan2(p.z(), p.x()) - theta0;
	jnt(0) = j0 * joint0.JointAxis().y();
	log(DEBUG) << "IK nose: theta0s = " << atan2(p.z(), p.x()) << ", theta0 = " << theta0 << ", j0 = " << jnt(0) <<  std::endl;

	//
	// calculate positions of JOINT2
	//
	double pxyz_norm2 = pxz_norm2 + p.y()*p.y();
	double cosTheta2s = - (pxyz_norm2 - l1*l1 - l2*l2) / (2.0*l1*l2);
	if (abs(cosTheta2s) > 1.000) {
		// out of rechability
		log(DEBUG) << "IK nose failed: out of rechability: l1 = " << l1 << ", l2 = " << l2 << ", Pxyz_norm = " << sqrt(pxyz_norm2) << endlog();
		return false;
	}
	double theta2s = acos( cosTheta2s );
	jnt(2) = M_PI - theta2s - theta2;
	jnt(2) *= joint2.JointAxis().y();
	log(DEBUG) << "IK nose: theta2 = " << theta2 << ", theta2s = " << theta2s <<  std::endl;

	//
	// calculate positions of JOINT1
	//
	double l3s = pxz_norm * sin(theta0);
	jnt(1) = atan2(p.y(), l3s);
	jnt(1) *= joint1.JointAxis().z();
	log(DEBUG) << "IK nose: l3s = " << l3s << std::endl;

	// 
	// calculate positions of JOINT3, JOINT4, JOINT5
	//
	Frame b_T_3s = joint0.pose(jnt(0)) * joint1.pose(jnt(1)) * joint2.pose(jnt(2)) * joint3.pose(0.0);
	b_T_3s.p -= joint0.JointOrigin();
	// check if result is sane
	if ( (b_T_3s.p - p).Norm() > tolerance_pos_ ) {
		log(DEBUG) << "IK nose: position tolerance exceeded: b_T3_s = " << b_T_3s << std::endl;
	}

	// transform axes: X -> Z, Z -> Y, Y -> X
	Rotation Rc = Rotation(0.0, 1.0, 0.0, 
	                      0.0, 0.0, 1.0, 
						  1.0, 0.0, 0.0);
	Rotation R = Rc*b_T_e.M.Inverse()*b_T_3s.M*Rc.Inverse();
	R.SetInverse();
	// calculate angles
	R.GetEulerZYX(jnt(3), jnt(4), jnt(5));
	// reverse signs if necessary
	jnt(3) *= joint3.JointAxis().x();
	jnt(4) *= joint4.JointAxis().z();
	jnt(5) *= joint5.JointAxis().y();

	// Debug output
	if (log(DEBUG)) {
		log() << "IK nose: b_T_3s = " << b_T_3s << " jnt345 = " << jnt.data.tail<3>().transpose() <<  std::endl;
		log() << "IK nose: b_R_3s = " << std::endl << b_T_3s.M.data[0] << b_T_3s.M.data[1] << b_T_3s.M.data[2] << std::endl << b_T_3s.M.data[3] << b_T_3s.M.data[4] << b_T_3s.M.data[5] << std::endl << b_T_3s.M.data[6] << b_T_3s.M.data[7] << b_T_3s.M.data[8] << std::endl;
		log() << "IK nose: R = " << std::endl << R.data[0] << R.data[1] << R.data[2] << std::endl << R.data[3] << R.data[4] << R.data[5] << std::endl << R.data[6] << R.data[7] << R.data[8] << std::endl;
		log() << endlog();
	}

	//
	// Limits check
	//
	if ( (jnt.data.array() > upper.data.array()).any()  || 
		 (jnt.data.array() < lower.data.array()).any() ) 
	{
		log(DEBUG) << "IK nose failed: joint limits: solution  " << jnt.data.transpose() << ", lower " << lower.data.transpose() << ", upper = " << upper.data.transpose() << endlog();
		return false;
	}

	return true;
}

}
}

