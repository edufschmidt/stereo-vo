#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>

struct Data {
	// number of mutually consistent feature pairs/matches to be considered by the optimization algorithm
	size_t n;

	// image coordinates (in the left image) of the features in frames a and b, respectively
	double *jax_data;
	double *jay_data;

	double *jbx_data;
	double *jby_data;

	// real-world coordinates of pixels ja and jb in the respective camera's coordinate frames
	double *wax_data;
	double *way_data;
	double *waz_data;

	double *wbx_data;
	double *wby_data;
	double *wbz_data;

	// constant matrix (row-wise) corresponding to the projective transformation of the left camera in rectified coordinates
    double *cam_mat_data;

    // array that indicates points that are considered outliers due to high reprojection error according to the previous R&T model
    bool *is_outlier_data;
};

double sqd_fwd_reprojection_error (double *jb, double *wa, double* r, double *t, double *cam_mat_data);
double sqd_bkwd_reprojection_error (double *ja, double *wb, double *r, double *t, double *cam_mat_data);
double bidirectional_reprojection_error (double *ja, double *jb, double *wa, double *wb, double* h, double *t, double *cam_mat_data);
int reprojection_error_fcn (const gsl_vector * params, void *data, gsl_vector * f);
gsl_multifit_fdfsolver * run_optimization(const int num_params, const int num_observations, Data *data, double *p0);
