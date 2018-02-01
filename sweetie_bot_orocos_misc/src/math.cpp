#include <sweetie_bot_orocos_misc/math.hpp>

namespace sweetie_bot {

void rotationMatrixToAngleAxis(const KDL::Rotation& m, KDL::Vector& axis, double& angle) {
	const double epsilon = 0.01; // margin to allow for rounding errors
	const double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees
	// optional check that input is pure rotation, 'isRotationMatrix' is defined at:
	// https://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
	if ((std::abs(m(0,1)-m(1,0)) < epsilon) && (std::abs(m(0,2)-m(2,0))< epsilon) && (std::abs(m(1,2)-m(2,1))< epsilon)) {
		// singularity found
		// first check for identity matrix which must have +1 for all terms
		//  in leading diagonaland zero in other terms
		if ((std::abs(m(0,1)+m(1,0)) < epsilon2) && (std::abs(m(0,2)+m(2,0)) < epsilon2) && (std::abs(m(1,2)+m(2,1)) < epsilon2) && (std::abs(m(0,0)+m(1,1)+m(2,2)-3) < epsilon2)) {
			// this singularity is identity matrix so angle = 0
			// zero angle, arbitrary axis
			angle = 0.0;
			axis = KDL::Vector(1,0,0);
			return;
		}
		// otherwise this singularity is angle = 180
		angle = 3.1415926f;
		double xx = (m(0,0)+1)/2;
		double yy = (m(1,1)+1)/2;
		double zz = (m(2,2)+1)/2;
		double xy = (m(0,1)+m(1,0))/4;
		double xz = (m(0,2)+m(2,0))/4;
		double yz = (m(1,2)+m(2,1))/4;
		if ((xx > yy) && (xx > zz)) { // m(0,0) is the largest diagonal term
			if (xx< epsilon) {
				axis = KDL::Vector(0, 0.7071, 0.7071);
			} else {
				double x = std::sqrt(xx);
				axis = KDL::Vector(x, xy/x, xz/x);
			}
		} else if (yy > zz) { // m(1,1) is the largest diagonal term
			if (yy< epsilon) {
				axis = KDL::Vector(0.7071, 0, 0.7071);
			} else {
				double y = std::sqrt(yy);
				axis = KDL::Vector(xy/y, y, yz/y);
			}	
		} else { // m(2,2) is the largest diagonal term so base result on this
			if (zz< epsilon) {
				axis = KDL::Vector(0.7071, 0.7071, 0);
			} else {
				double z = std::sqrt(zz);
				axis = KDL::Vector(xz/z, yz/z, z);
			}
		}
		// return 180 deg rotation
		return;	
	}
	// as we have reached here there are no singularities so we can handle normally
	double s = std::sqrt((m(2,1) - m(1,2))*(m(2,1) - m(1,2))
		+(m(0,2) - m(2,0))*(m(0,2) - m(2,0))
		+(m(1,0) - m(0,1))*(m(1,0) - m(0,1))); // used to normalise
	if (std::abs(s) < 0.001) s=1; 
		// prevent divide by zero, should not happen if matrix is orthogonal and should be
		// caught by singularity test above, but I've left it in just in case
	angle = std::acos((m(0,0) + m(1,1) + m(2,2) - 1)/2);
	axis = KDL::Vector((m(2,1) - m(1,2))/s, (m(0,2) - m(2,0))/s, (m(1,0) - m(0,1))/s);
}


} // namespace sweetie_bot
