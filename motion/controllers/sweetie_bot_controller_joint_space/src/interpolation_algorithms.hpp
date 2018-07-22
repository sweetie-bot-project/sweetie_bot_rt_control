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
		 * @brief Perform spline enterpolation for a set of points' arrays of same size.
		 *
		 * Functions does not checks lenghts of points arrays! Size of @a t and arrays in @points must be equal.
		 * Null poinaters are not checked.
		 * @param t time @c alglib::real_1d_array.
		 * @param trajectoties the pointer to the array of alglib::real_1d_array arrays of points. There maust be at least @a n @c alglib::real_1d_array arrays.
		 * @param splines the pointer to result splines array. Its length must be not less tnen @a n.
		 * @param n_trajectories number of points' arrays.
		 */
		virtual void performInterpolation(const real_1d_array& t, const real_1d_array * trajectoties, spline1dinterpolant * splines, int n_trajectories) const = 0;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class AkimaSplineInterpolation : public InterpolationAlgorithmInterface {
	public:
		void performInterpolation(const real_1d_array& t, const real_1d_array * trajectoties, spline1dinterpolant * splines, int n_trajectories) const;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class CubicSplineInterpolation : public InterpolationAlgorithmInterface {
	public:
		void performInterpolation(const real_1d_array& t, const real_1d_array * trajectoties, spline1dinterpolant * splines, int n_trajectories) const;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on Akima spline
 **/
class ModifiedAkimaInterpolation : public InterpolationAlgorithmInterface {
	protected:
		double threshold;

	public:
		ModifiedAkimaInterpolation(double _threshold) : threshold(_threshold) {}

		void performInterpolation(const real_1d_array& t, const real_1d_array * trajectoties, spline1dinterpolant * splines, int n_trajectories) const;
};

/**
 * @brief Incapsulate implementation of interpolation algorithm based on cubic spline
 **/
class ModifiedCubicInterpolation : public InterpolationAlgorithmInterface {
	protected:
		double threshold;

	public:
		ModifiedCubicInterpolation(double _threshold) : threshold(_threshold) {}

		void performInterpolation(const real_1d_array& t, const real_1d_array * trajectoties, spline1dinterpolant * splines, int n_trajectories) const;
};

} // namespace controller
} // namespace motion
} // namespace sweetie_bot

#endif  /* INTERPOLATION_ALGORITHMS_HPP */
