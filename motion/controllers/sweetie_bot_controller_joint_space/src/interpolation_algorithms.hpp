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
	ModifyAkima,
	ModifyCubic
};

/**
 * @brief Providing interface to select algorithm
 **/
class InterpolationAlgorithm {
	public:
		typedef alglib::spline1dinterpolant JointSpline;

	protected:
		double threshold;

	public:
		InterpolationAlgorithm(double threshold) { this->threshold = threshold; }

		virtual void performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints) {}
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class ModifyAkimaAlgorithm : public InterpolationAlgorithm {
	public:
		ModifyAkimaAlgorithm(double threshold)
			: InterpolationAlgorithm(threshold) {}

		void performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints);
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on cubic spline
 **/
class ModifyCubicAlgorithm : public InterpolationAlgorithm {
	public:
		ModifyCubicAlgorithm(double threshold)
			: InterpolationAlgorithm(threshold) {}

		void performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints);
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /* INTERPOLATION_ALGORITHMS_HPP */
