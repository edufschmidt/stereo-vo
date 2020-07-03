
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>
#include <BidirectionalReprojectionErrorOptUtils.hpp>

double sqd_fwd_reprojection_error (double *jb, double *wa, double* r, double *t, double *cam_mat_data){

		double tx=t[0], ty=t[1], tz=t[2];

		double jbx = jb[0], jby = jb[1];

		double wax = wa[0], way = wa[1], waz = wa[2];

		double p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34;
			p11=cam_mat_data[0]; p12=cam_mat_data[1]; p13=cam_mat_data[2];  p14=cam_mat_data[3];
			p21=cam_mat_data[4]; p22=cam_mat_data[5]; p23=cam_mat_data[6];  p24=cam_mat_data[7];
			p31=cam_mat_data[8]; p32=cam_mat_data[9]; p33=cam_mat_data[10]; p34=cam_mat_data[11];

		// auxiliary variables
		double r11, r12, r13, r21, r22, r23, r31, r32, r33;
		double proj_wa11, proj_wa21, proj_wa31;

		r11 = r[0]; r12 = r[1]; r13 = r[2];
		r21 = r[3]; r22 = r[4]; r23 = r[5];
		r31 = r[6]; r32 = r[7]; r33 = r[8];

		// PROJECTION OF POINTS FROM FRAME 'A' TO FRAME 'B' (FORWARD)
		proj_wa11 = p14 + p11*tx + p12*ty + p13*tz + wax*(p11*r11 + p12*r21 + p13*r31) + way*(p11*r12 + p12*r22 + p13*r32) + waz*(p11*r13 + p12*r23 + p13*r33);
 		proj_wa21 = p24 + p21*tx + p22*ty + p23*tz + wax*(p21*r11 + p22*r21 + p23*r31) + way*(p21*r12 + p22*r22 + p23*r32) + waz*(p21*r13 + p22*r23 + p23*r33);
 		proj_wa31 = p34 + p31*tx + p32*ty + p33*tz + wax*(p31*r11 + p32*r21 + p33*r31) + way*(p31*r12 + p32*r22 + p33*r32) + waz*(p31*r13 + p32*r23 + p33*r33);

		// normalize proj_wa wrt the last coordinate (bring it back from homogeneous repr.)
		proj_wa11 /= proj_wa31;
		proj_wa21 /= proj_wa31;
		proj_wa31 /= proj_wa31;

		#ifdef DEBUG
			printf("Point jb = (%lf, %lf), reprojection proj_wa = (%lf,%lf)\n", jbx, jby, proj_wa11, proj_wa21);
		#endif

	    double sqd_fwd_reproj_err  = (pow(jbx - proj_wa11,2) + pow(jby - proj_wa21,2)); // residual/error in projection from frame a to frame b

		return sqd_fwd_reproj_err;
}

double sqd_bkwd_reprojection_error (double *ja, double *wb, double *r, double *t, double *cam_mat_data){

		double jax = ja[0], jay = ja[1];

		double wbx = wb[0], wby = wb[1], wbz = wb[2];

		double tx = t[0], ty = t[1], tz = t[2];

		double p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34;
			p11=cam_mat_data[0]; p12=cam_mat_data[1]; p13=cam_mat_data[2];  p14=cam_mat_data[3];
			p21=cam_mat_data[4]; p22=cam_mat_data[5]; p23=cam_mat_data[6];  p24=cam_mat_data[7];
			p31=cam_mat_data[8]; p32=cam_mat_data[9]; p33=cam_mat_data[10]; p34=cam_mat_data[11];

		double r11, r12, r13, r21, r22, r23, r31, r32, r33;

		double proj_wb11, proj_wb21, proj_wb31;

		r11 = r[0]; r12 = r[1]; r13 = r[2];
		r21 = r[3]; r22 = r[4]; r23 = r[5];
		r31 = r[6]; r32 = r[7]; r33 = r[8];

		// projection of points from frame 'B' to frame 'A' (backward)
		proj_wb11 = (p14*r11*r22*r33 - p14*r11*r23*r32 - p14*r12*r21*r33 + p14*r12*r23*r31 + p14*r13*r21*r32 - p14*r13*r22*r31 - p11*r22*r33*tx + p11*r23*r32*tx + p12*r21*r33*tx - p12*r23*r31*tx - p13*r21*r32*tx + p13*r22*r31*tx + p11*r12*r33*ty - p11*r13*r32*ty - p12*r11*r33*ty + p12*r13*r31*ty + p13*r11*r32*ty - p13*r12*r31*ty - p11*r12*r23*tz + p11*r13*r22*tz + p12*r11*r23*tz - p12*r13*r21*tz - p13*r11*r22*tz + p13*r12*r21*tz + p11*r22*r33*wbx - p11*r23*r32*wbx - p12*r21*r33*wbx + p12*r23*r31*wbx + p13*r21*r32*wbx - p13*r22*r31*wbx - p11*r12*r33*wby + p11*r13*r32*wby + p12*r11*r33*wby - p12*r13*r31*wby - p13*r11*r32*wby + p13*r12*r31*wby + p11*r12*r23*wbz - p11*r13*r22*wbz - p12*r11*r23*wbz + p12*r13*r21*wbz + p13*r11*r22*wbz - p13*r12*r21*wbz)/(r11*r22*r33 - r11*r23*r32 - r12*r21*r33 + r12*r23*r31 + r13*r21*r32 - r13*r22*r31);
		proj_wb21 = (p24*r11*r22*r33 - p24*r11*r23*r32 - p24*r12*r21*r33 + p24*r12*r23*r31 + p24*r13*r21*r32 - p24*r13*r22*r31 - p21*r22*r33*tx + p21*r23*r32*tx + p22*r21*r33*tx - p22*r23*r31*tx - p23*r21*r32*tx + p23*r22*r31*tx + p21*r12*r33*ty - p21*r13*r32*ty - p22*r11*r33*ty + p22*r13*r31*ty + p23*r11*r32*ty - p23*r12*r31*ty - p21*r12*r23*tz + p21*r13*r22*tz + p22*r11*r23*tz - p22*r13*r21*tz - p23*r11*r22*tz + p23*r12*r21*tz + p21*r22*r33*wbx - p21*r23*r32*wbx - p22*r21*r33*wbx + p22*r23*r31*wbx + p23*r21*r32*wbx - p23*r22*r31*wbx - p21*r12*r33*wby + p21*r13*r32*wby + p22*r11*r33*wby - p22*r13*r31*wby - p23*r11*r32*wby + p23*r12*r31*wby + p21*r12*r23*wbz - p21*r13*r22*wbz - p22*r11*r23*wbz + p22*r13*r21*wbz + p23*r11*r22*wbz - p23*r12*r21*wbz)/(r11*r22*r33 - r11*r23*r32 - r12*r21*r33 + r12*r23*r31 + r13*r21*r32 - r13*r22*r31);
		proj_wb31 = (p34*r11*r22*r33 - p34*r11*r23*r32 - p34*r12*r21*r33 + p34*r12*r23*r31 + p34*r13*r21*r32 - p34*r13*r22*r31 - p31*r22*r33*tx + p31*r23*r32*tx + p32*r21*r33*tx - p32*r23*r31*tx - p33*r21*r32*tx + p33*r22*r31*tx + p31*r12*r33*ty - p31*r13*r32*ty - p32*r11*r33*ty + p32*r13*r31*ty + p33*r11*r32*ty - p33*r12*r31*ty - p31*r12*r23*tz + p31*r13*r22*tz + p32*r11*r23*tz - p32*r13*r21*tz - p33*r11*r22*tz + p33*r12*r21*tz + p31*r22*r33*wbx - p31*r23*r32*wbx - p32*r21*r33*wbx + p32*r23*r31*wbx + p33*r21*r32*wbx - p33*r22*r31*wbx - p31*r12*r33*wby + p31*r13*r32*wby + p32*r11*r33*wby - p32*r13*r31*wby - p33*r11*r32*wby + p33*r12*r31*wby + p31*r12*r23*wbz - p31*r13*r22*wbz - p32*r11*r23*wbz + p32*r13*r21*wbz + p33*r11*r22*wbz - p33*r12*r21*wbz)/(r11*r22*r33 - r11*r23*r32 - r12*r21*r33 + r12*r23*r31 + r13*r21*r32 - r13*r22*r31);

		// normalize proj_wb wrt the last coordinate (bring it back from homogeneous repr.)
		proj_wb11 /= proj_wb31;
		proj_wb21 /= proj_wb31;
		proj_wb31 /= proj_wb31;

		#ifdef DEBUG
			printf("Point ja = (%lf, %lf), reprojection proj_wb = (%lf,%lf)\n", jax, jay, proj_wb11, proj_wb21);
		#endif

	    double sqd_bkwd_reproj_err = (pow(jax - proj_wb11,2) + pow(jay - proj_wb21,2)); // residual/error in projection from frame b to frame a

	    return sqd_bkwd_reproj_err;
}

double bidirectional_reprojection_error (double *ja, double *jb, double *wa, double *wb, double* h, double *t, double *cam_mat_data){

	double w, x, y, z;
	w=h[0]; x=h[1]; y=h[2]; z=h[3];

	double n = w*w + x*x + y*y + z*z;
	double s;
		if(n == 0) s=0; else s = 2/n;
	double wx = s*w*x, wy = s*w*y, wz = s*w*z;
	double xx = s*x*x, xy = s*x*y, xz = s*x*z;
	double yy = s*y*y, yz = s*y*z, zz = s*z*z;

	double r11 = 1 - (yy + zz);
	double r21 = xy + wz;
	double r31 = xz - wy;

	double r12 = xy - wz;
	double r22 = 1- (xx + zz);
	double r32 = yz + wx;

	double r13 = xz + wy;
	double r23 = yz - wx;
	double r33 = 1 - (xx + yy);

	double r[] = {r11, r12, r13,
				  r21, r22, r23,
				  r31, r32, r33};

	double bidirect_reproj_err =  sqd_fwd_reprojection_error(jb, wa, r, t, cam_mat_data) + sqd_bkwd_reprojection_error(ja, wb, r, t, cam_mat_data);

	return bidirect_reproj_err;
}

int reprojection_error_fcn (const gsl_vector * params, void *data, gsl_vector * f){
	// retrieve all data from the struct passed as an argument to the function
	size_t num_matches = ((struct Data *)data)->n;
	// feature image coordinates on frame 'A'
	double *jax_data = ((struct Data *)data)->jax_data;	double *jay_data = ((struct Data *)data)->jay_data;
	// feature image coordinates on frame 'B'
	double *jbx_data = ((struct Data *)data)->jbx_data;	double *jby_data = ((struct Data *)data)->jby_data;
	// real world coordinates wrt frame 'A' coordinates system
	double *wax_data = ((struct Data *)data)->wax_data;	double *way_data = ((struct Data *)data)->way_data;	double *waz_data = ((struct Data *)data)->waz_data;
	// real world coordinates wrt frame 'B' coordinates system
	double *wbx_data = ((struct Data *)data)->wbx_data;	double *wby_data = ((struct Data *)data)->wby_data;	double *wbz_data = ((struct Data *)data)->wbz_data;
	// camera projection matrix (on the rectified frame)
	double *cam_mat_data  = ((struct Data *)data)->cam_mat_data;
	// array that indicates whether the feature pair was marked as a 'remaining' outlier (one that survived the outlier removal step but still has high reprojection error)
	bool *is_outlier_data  = ((struct Data *)data)->is_outlier_data;

	// get rotation quaternion from the params
	double w = gsl_vector_get (params, 0);
	double x = gsl_vector_get (params, 1);
	double y = gsl_vector_get (params, 2);
	double z = gsl_vector_get (params, 3);

	// get translation vector from the params
	double tx = gsl_vector_get (params, 4);
	double ty = gsl_vector_get (params, 5);
	double tz = gsl_vector_get (params, 6);

	// compute rotation matrix entries from the quaternion
	double n = w*w + x*x + y*y + z*z;
	double s;
		if(n == 0) s=0; else s = 2/n;
	double wx = s*w*x, wy = s*w*y, wz = s*w*z;
	double xx = s*x*x, xy = s*x*y, xz = s*x*z;
	double yy = s*y*y, yz = s*y*z, zz = s*z*z;

	double r11 = 1 - (yy + zz);
	double r21 = xy + wz;
	double r31 = xz - wy;

	double r12 = xy - wz;
	double r22 = 1- (xx + zz);
	double r32 = yz + wx;

	double r13 = xz + wy;
	double r23 = yz - wx;
	double r33 = 1 - (xx + yy);

	double r[] = {r11, r12, r13,
				  r21, r22, r23,
				  r31, r32, r33};

	double t[] = {tx, ty, tz};

    double ja[2];
    double jb[2];
    double wa[3];
    double wb[3];

   for (size_t i = 0; i < num_matches; i++){
     	ja[0] = jax_data[i]; ja[1] = jay_data[i];
		jb[0] = jbx_data[i]; jb[1] = jby_data[i];
		wa[0] = wax_data[i]; wa[1] = way_data[i]; wa[2] = waz_data[i];
		wb[0] = wbx_data[i]; wb[1] = wby_data[i]; wb[2] = wbz_data[i];

	    double fwd_reproj_err  = sqrt(sqd_fwd_reprojection_error(jb, wa, r, t, cam_mat_data));   // residual/error in projection from frame a to frame b
	    double bkwd_reproj_err = sqrt(sqd_bkwd_reprojection_error(ja, wb, r, t, cam_mat_data)); // residual/error in projection from frame b to frame a

	    // check whether the current feature pair is marked as an outlier due to high reprojection error in previous passes of the LM alg.
  		if(is_outlier_data[i]){
  			//cout << "Match "<< i <<" is an outlier. Discarding..." << std::endl;
  			gsl_vector_set (f, i, 0.0);
  			gsl_vector_set (f, (2*num_matches-1)-i, 0.0);
  		} else {
  			gsl_vector_set (f, i, fwd_reproj_err);
  			gsl_vector_set (f, (2*num_matches-1)-i, bkwd_reproj_err);
  		}
   }

  	return GSL_SUCCESS;
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

gsl_multifit_fdfsolver * run_optimization(const int num_params, const int num_observations, Data *data, double *p0){

		gsl_multifit_function_fdf function;
		double initial_params[] = {p0[0], p0[1], p0[2], p0[3], p0[4], p0[5], p0[6]};
		gsl_vector_view params = gsl_vector_view_array (initial_params, num_params);

		const gsl_rng_type * type;
		gsl_rng * r;

		gsl_rng_env_setup();

		type = gsl_rng_default;
		r = gsl_rng_alloc (type);

		function.f = &reprojection_error_fcn;
		function.df = NULL;
		function.fdf = NULL;
		function.n = num_observations;
		function.p = num_params;
		function.params = data;

		int status;
		unsigned int iter = 0;
		const gsl_multifit_fdfsolver_type * T = gsl_multifit_fdfsolver_lmder; // return the type corresponding to a LM solver
		gsl_multifit_fdfsolver * solver = gsl_multifit_fdfsolver_alloc (T, num_observations, num_params);  // allocate the solver
	 	gsl_multifit_fdfsolver_set (solver, &function, &params.vector);

	  	do{
		      iter++;
		      status = gsl_multifit_fdfsolver_iterate (solver);

		      if (status)
		        break;

		      status = gsl_multifit_test_delta(solver->dx, solver->x, 1e-5, 1e-5);

	    } while (status == GSL_CONTINUE && iter < 500);

	    return solver;
}
