#ifndef  SWEETIE_BOT_OROCOS_MISC_MATH_HPP
#define  SWEETIE_BOT_OROCOS_MISC_MATH_HPP

#include <geometry_msgs/Quaternion.h>
#include <kdl/frames.hpp>

namespace sweetie_bot {


/**
 * @brief Normalize quaternion message inplace.
 * |x|^2 + |y|^2 + |z|^2 + |w|^2 = 1
 * @param q Quaternion to be normalized.
 **/
inline void normalizeQuaternionMsg(geometry_msgs::Quaternion& q) {
	auto s = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
	q.x /= s; q.y /= s; q.z /= s; q.w /= s;
}

/**
 * @brief Convert rotation matrix to angle-axis representation.
 * It is C++ adaptation of following routine:
 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
 * @param m Rotation matrix.
 * @param axis Calculated rotation axis.
 * @param angle Calculated rotation angle.
 **/
void rotationMatrixToAngleAxis(const KDL::Rotation& m, KDL::Vector& axis, double& angle);

/**
 * @brief Calculate logarithm of rotation matrix.
 * This function returns coresponding 3-element vector (x, y z), coresponding 
 * screw-symmetric matrix can be calculated by formula
 * @code
 *     [ 0  -z y  ]
 * S = [ z  0  -x ]
 *     [ -y x  0  ]
 * @endcode
 * @param m Rotation matrix.
 **/
inline KDL::Vector rotationMatrixLogarithm(const KDL::Rotation& m) {
	KDL::Vector axis;
	double angle;
	rotationMatrixToAngleAxis(m, axis, angle);
	return angle * axis;
}


} // namespace sweetie_bot

#endif  /*SWEETIE_BOT_OROCOS_MISC_MATH_HPP*/
