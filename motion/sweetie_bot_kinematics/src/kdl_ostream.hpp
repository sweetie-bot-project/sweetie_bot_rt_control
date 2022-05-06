#include <ostream>

#ifndef  KDL_OSTREAM_HPP
#define  KDL_OSTREAM_HPP

inline std::ostream& operator<<(std::ostream& s, const KDL::Vector& v) 
{
	s << "[" << v.x() << " " << v.y() << " " << v.z() << " ]";
	return s;
}

inline std::ostream& operator<<(std::ostream& s, const KDL::Twist& v) 
{
	s << "[ rot = " << v.rot << ", vel = " << v.vel << " ]";
	return s;
}
inline std::ostream& operator<<(std::ostream& s, const KDL::Rotation& R) 
{
	KDL::Vector rpy;
	R.GetRPY(rpy.data[0], rpy.data[1], rpy.data[2]);
	s << "RPY = " << rpy << std::endl;
	return s;
}
inline std::ostream& operator<<(std::ostream& s, const KDL::Frame& T) { 
	s << "[ p = " << T.p <<  ", " << T.M  << " ]"<< std::endl;
	return s;
}
#endif  /*KDL_OSTREAM_HPP*/
