#ifndef  STREAM_OPERATORS_HPP
#define  STREAM_OPERATORS_HPP

#include <ostream>
#include <vector>
#include <kdl/frames.hpp>

template <typename T>
std::ostream& operator<<(std::ostream& oss, const std::vector<T>& vec)
{
	oss << "[ ";
	for(const T& elem : vec) oss << elem << ", ";
	oss << "]";
	return oss;
}

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

/*inline std::ostream& operator<<(std::ostream& s, const KDL::Rotation& R)
{
	s << std::endl;
	s << R(0,0) << " " << R(0,1) << " " << R(0,2) << std::endl;
	s << R(1,0) << " " << R(1,1) << " " << R(1,2) << std::endl;
	s << R(2,0) << " " << R(2,1) << " " << R(2,2) << std::endl;
	return s;
}*/

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

#endif  /*STREAM_OPERATORS_HPP*/
