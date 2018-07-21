#include "interpolation_algorithms.hpp"

#include <rtt/RTT.hpp>

namespace sweetie_bot {
namespace motion {
namespace controller {

/**
 * Implements algorithm for spline interpolation based on Akima spline
 **/
void ModifiedAkimaInterpolation::performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints) const {
	JointSpline akima_tmp_spline;
	alglib::real_1d_array d;
	double dirty, diff;
	for(int joint = 0; joint < n_joints; joint++) {
		//
		// Build custom akima spline with zero velocity at edge points
		// In order to do this calculate derivatives from simple akima spline in each interpolation point
		//
		alglib::spline1dbuildakima(t, joint_trajectory[joint], akima_tmp_spline);

		// While calculating derivatives detect almost equal internal stop points 
		d.setlength(n_samples);
		for (int i = 0; i < (n_samples - 1); i++) {
			// Detect internal stop points with use of selected threshold
			if (abs(joint_trajectory[joint][i + 1] - joint_trajectory[joint][i]) <= this->threshold) {
				// For detected points derivative (velocity) will be zero
				d[i] = 0.0;
				d[i + 1] = 0.0;
			} else {
				alglib::spline1ddiff(akima_tmp_spline, t[i], dirty, d[i], dirty);
			}
		}

		// Set up first and last point to 0 velocity in order to avoid an outliers at the edges of trajectory
		d[0] = 0.0;
		d[n_samples - 1] = 0.0;

		// Putting it all together with use of hermite spline building function
		alglib::spline1dbuildhermite(t, joint_trajectory[joint], d, joint_splines[joint]);
	}
}

/**
 * Implements algorithm for spline interpolation based on cubic
 **/
void ModifiedCubicInterpolation::performInterpolation(const alglib::real_1d_array& t, const std::vector<alglib::real_1d_array>& joint_trajectory, double n_samples, std::vector<JointSpline>& joint_splines, double n_joints) const {
	int len, i, j, kk;
	int _1st_point_idx;
	int _2nd_point_idx;
	const alglib::real_1d_array *joint_t;
	alglib::real_1d_array _x;
	alglib::real_1d_array _y;
	alglib::spline1dinterpolant s_cubic;
	alglib_impl::spline1dinterpolant *s_result;
	alglib_impl::ae_state _aeState;

	// Initialize variables
	_x.setlength(n_samples);
	_y.setlength(n_samples);
	alglib_impl::ae_state_init(&_aeState);
	for(int joint = 0; joint < n_joints; joint++) {
		// Initialize variables for iteration
		kk = 0;
		_1st_point_idx = 0;
		_2nd_point_idx = 0;
		joint_t = &joint_trajectory[joint];
		s_result = const_cast<alglib_impl::spline1dinterpolant*>(joint_splines[joint].c_ptr());

		s_result->n = n_samples;
		s_result->k = 3;
		s_result->periodic = ae_false;
		s_result->continuity = 2;
		alglib_impl::ae_vector_set_length(&s_result->x, s_result->n, &_aeState);
		alglib_impl::ae_vector_set_length(&s_result->c, 4*(s_result->n-1)+2, &_aeState);
		do {
			//
			//  Changing points range process
			//

			// Looking for desired range
			while (_2nd_point_idx < (n_samples - 1) && abs((*joint_t)[_2nd_point_idx] - (*joint_t)[_2nd_point_idx + 1]) > this->threshold) {
				_2nd_point_idx++;
			}

			if (_2nd_point_idx < n_samples && _2nd_point_idx - _1st_point_idx > 0) {
				// Building cubic spline for current range
				len = _2nd_point_idx - _1st_point_idx + 1;
				for (i = _1st_point_idx, j = 0; i <= _2nd_point_idx; i++, j++) {
					_x[j] = t[i];
					_y[j] = (*joint_t)[i];
				}
				alglib::spline1dbuildcubic(_x, _y, len, 1, 0.0, 1, 0.0, s_cubic);

				// Assembling interpolant for current range with data from cubic spline
				if (_1st_point_idx == 0) {
					s_result->x.ptr.p_double[0] = t[0];
				}
				for (i = _1st_point_idx; i < _2nd_point_idx; i++, kk++) {
					s_result->x.ptr.p_double[kk+1] = t[i+1];
					s_result->c.ptr.p_double[(s_result->k+1)*kk+0] = s_cubic.c_ptr()->c.ptr.p_double[(s_result->k+1)*(i-_1st_point_idx)+0];
					s_result->c.ptr.p_double[(s_result->k+1)*kk+1] = s_cubic.c_ptr()->c.ptr.p_double[(s_result->k+1)*(i-_1st_point_idx)+1];
					s_result->c.ptr.p_double[(s_result->k+1)*kk+2] = s_cubic.c_ptr()->c.ptr.p_double[(s_result->k+1)*(i-_1st_point_idx)+2];
					s_result->c.ptr.p_double[(s_result->k+1)*kk+3] = s_cubic.c_ptr()->c.ptr.p_double[(s_result->k+1)*(i-_1st_point_idx)+3];
				}

				_1st_point_idx = _2nd_point_idx++;
			}

			//
			//  Equal points range process
			//

			// Looking for desired range
			while (_2nd_point_idx < (n_samples - 1) && abs((*joint_t)[_2nd_point_idx] - (*joint_t)[_2nd_point_idx + 1]) <= this->threshold) {
				_2nd_point_idx++;
			}

			if (_2nd_point_idx < n_samples) {
				// Assembling interpolant for current range manually assigning spline coefficients
				if (_1st_point_idx == 0) {
					s_result->x.ptr.p_double[0] = t[0];
				}
				for (i = _1st_point_idx; i < _2nd_point_idx; i++, kk++) {
					s_result->x.ptr.p_double[kk+1] = t[i+1];
					s_result->c.ptr.p_double[(s_result->k+1)*kk+0] = (*joint_t)[i];
					s_result->c.ptr.p_double[(s_result->k+1)*kk+1] = 0;
					s_result->c.ptr.p_double[(s_result->k+1)*kk+2] = 0;
					s_result->c.ptr.p_double[(s_result->k+1)*kk+3] = 0;
				}

				_1st_point_idx = _2nd_point_idx++;
			}
		} while (_2nd_point_idx < n_samples);

		// Insertion last two values to resulting spline
		s_result->c.ptr.p_double[4*(s_result->n-1)+0] = (*joint_t)[s_result->n-1];
		s_result->c.ptr.p_double[4*(s_result->n-1)+1] = 0;
	}
	alglib_impl::ae_state_clear(&_aeState);
}

} // namespace controller
} // namespace motion
} // namespace sweetie_bot
