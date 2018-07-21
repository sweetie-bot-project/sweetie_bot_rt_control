#ifndef INTERPOLATION_ALGORITHMS_HPP
#define INTERPOLATION_ALGORITHMS_HPP

#include <interpolation.h>

namespace sweetie_bot {
namespace motion {
namespace controller {

/*
 * @brief Enum for separate algorithm types
 */
enum InterpolationAlgorithmType {
	ModifiedAkima = 0,
	ModifiedCubic = 1,
};

/**
 * @brief Providing interface to select algorithm
 **/
class InterpolationAlgorithmInterface {
	public:
		typedef alglib::spline1dinterpolant JointSpline;

	public:
		virtual void performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints) const = 0;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class ModifiedAkimaInterpolation : public InterpolationAlgorithmInterface {
	protected:
		double threshold;

	public:
		ModifiedAkimaInterpolation(double _threshold) : threshold(_threshold) {}

		void performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints) const;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on cubic spline
 **/
class ModifiedCubicInterpolation : public InterpolationAlgorithmInterface {
	protected:
		double threshold;

	public:
		ModifiedCubicInterpolation(double _threshold) : threshold(_threshold) {}

		void performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints) const;
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /* INTERPOLATION_ALGORITHMS_HPP */
