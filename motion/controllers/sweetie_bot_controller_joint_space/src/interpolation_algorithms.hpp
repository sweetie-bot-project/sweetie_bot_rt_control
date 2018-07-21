#ifndef INTERPOLATION_ALGORITHMS_HPP
#define INTERPOLATION_ALGORITHMS_HPP

#include <interpolation.h>

namespace sweetie_bot {
namespace motion {
namespace controller {

/**
 * @brief Providing interface to select algorithm
 **/
class InterpolationAlgorithmInterface {
	public:
		typedef alglib::spline1dinterpolant spline1dinterpolant;
		typedef alglib::real_1d_array real_1d_array;

	public:
		/**
		 * @brief Construct alglib::spline1dinterpolant for given trajectories.
		 * Functions does not checks lenghts of points arrays! Size of @a t and arrays in @points must be equal.
		 * @param t time alglib::real_1d_array, may be unsorted.
		 * @param points vector of alglib::real_1d_array trajectory points.
		 * @param splines vector of result splines it is resized according to @a points size.
		 */
		virtual void performInterpolation(const real_1d_array& t, const std::vector<real_1d_array>& points, std::vector<spline1dinterpolant>& splines) const = 0;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class AkimaSplineInterpolation : public InterpolationAlgorithmInterface {
	public:
		void performInterpolation(const real_1d_array& t, const std::vector<real_1d_array>& joint_trajectory, std::vector<spline1dinterpolant>& joint_splines) const;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class CubicSplineInterpolation : public InterpolationAlgorithmInterface {
	public:
		void performInterpolation(const real_1d_array& t, const std::vector<real_1d_array>& joint_trajectory, std::vector<spline1dinterpolant>& joint_splines) const;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class ModifiedAkimaInterpolation : public InterpolationAlgorithmInterface {
	protected:
		double threshold;

	public:
		ModifiedAkimaInterpolation(double _threshold) : threshold(_threshold) {}

		void performInterpolation(const real_1d_array& t, const std::vector<real_1d_array>& joint_trajectory, std::vector<spline1dinterpolant>& joint_splines) const;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on cubic spline
 **/
class ModifiedCubicInterpolation : public InterpolationAlgorithmInterface {
	protected:
		double threshold;

	public:
		ModifiedCubicInterpolation(double _threshold) : threshold(_threshold) {}

		void performInterpolation(const real_1d_array& t, const std::vector<real_1d_array>& joint_trajectory, std::vector<spline1dinterpolant>& joint_splines) const;
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /* INTERPOLATION_ALGORITHMS_HPP */
