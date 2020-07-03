#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlin.h>

#include <Frame.hpp>
#include <HowardMotionEstimator.hpp>
#include <FeatureMatchingUtils.hpp>
#include <BidirectionalReprojectionErrorOptUtils.hpp>

// ------------------------------------------------------------------------------------------------------------------------

        // Auxiliary function to perform the feature detection, discarding invalid points according
        // to their location and disparity/depth value.
        void HowardMotionEstimator::detectValidFeatures(std::shared_ptr<Frame> leftFrame, DepthSource *depthSource, AbstractFeatureDetector *detector){

            detector->detectFeatures(leftFrame);

            std::vector<KeyPoint> keypointsWithValid3D = std::vector<KeyPoint>();
            for(size_t i=0; i < leftFrame->keypointsOnImageCoordinates.size(); i++){
                KeyPoint feat = leftFrame->keypointsOnImageCoordinates[i];
                cv::Vec3f coord3D = depthSource->getWorldCoordinates(feat.pt.y, feat.pt.x);
                if(coord3D[0] != -1){ // having coord3D[j] equals to -1 means it's not a valid 3D coordinate
                    keypointsWithValid3D.push_back(feat);
                    leftFrame->keypointsOnWorldCoordinates.push_back(coord3D);
                }
            }
            leftFrame->keypointsOnImageCoordinates = keypointsWithValid3D;
        }

// ------------------------------------------------------------------------------------------------------------------------

        cv::Mat HowardMotionEstimator::buildConsistencyMatrix(std::vector<cv::DMatch> all_matches, double distanceDelta){
            // distanceDelta is in the same unit used in the calibration
            cv::Mat W(all_matches.size(), all_matches.size(), CV_8UC1);

            DMatch match_i, match_j;
            int idx_feat_frame_a, idx_feat_frame_b;
            int idx_feat2_frame_a, idx_feat2_frame_b;
            Vec3f coords_feat_a, coords_feat2_a;
            Vec3f coords_feat_b, coords_feat2_b;
            Vec3f dist_fa, dist_fb;

            for(int i=0; i<W.rows; i++){
            match_i = all_matches[i];
            idx_feat_frame_a = match_i.queryIdx;
            idx_feat_frame_b = match_i.trainIdx;

                for(int j=0; j<W.cols; j++){

                    match_j = all_matches[j];
                    idx_feat2_frame_a = match_j.queryIdx;
                    idx_feat2_frame_b = match_j.trainIdx;

                    coords_feat_a = this->leftFrameA->keypointsOnWorldCoordinates[idx_feat_frame_a];
                    coords_feat_b = this->leftFrameB->keypointsOnWorldCoordinates[idx_feat_frame_b];

                    coords_feat2_a = this->leftFrameA->keypointsOnWorldCoordinates[idx_feat2_frame_a];
                    coords_feat2_b = this->leftFrameB->keypointsOnWorldCoordinates[idx_feat2_frame_b];

                    double distance_diff = norm(coords_feat_a - coords_feat2_a) - norm(coords_feat_b-coords_feat2_b);

                    if(abs(distance_diff) < distanceDelta){
                        W.at<uchar>(i,j) = 1;
                    }else{
                        W.at<uchar>(i,j) = 0;
                    }
                }
            }
        return W;
        }

// ------------------------------------------------------------------------------------------------------------------------

       std::vector<DMatch> HowardMotionEstimator::selectConsistentMatches(std::vector<DMatch> matches, double distanceDelta){

            cv::Mat W = this->buildConsistencyMatrix(matches, distanceDelta);

            // Inlier set to be computed
            std::vector<DMatch> inliers;
            std::vector<uint> Q;
            std::vector<uint> compatible_matches;

            // Sum all columns of W
            cv::Mat sumW;
            cv::reduce(W, sumW, 1, CV_REDUCE_SUM, CV_64FC1); //sum all columns of W -> equivalent to getting the degree of each node
            cv::transpose(sumW, sumW);

            double min_dbl, max_dbl, best_match_score;
            cv::Point min_loc, max_loc;
            uint idx_max, max, best_match_idx;

            cv::minMaxLoc(sumW, &min_dbl, &max_dbl, &min_loc, &max_loc); //find the index and degree of the node with the maximum degree
            max = (uint) max_dbl; // degree (number of other nodes to which this node is connected)
            idx_max = (uint) max_loc.x; // index of the node with the maximum degree

            Q.push_back(idx_max);

            bool is_consistent_with_all_in_Q;

            while(true){
                compatible_matches.clear();
                for(size_t i=0; i<(size_t)W.rows; i++){
                    is_consistent_with_all_in_Q = true;

                    for(size_t j=0; j<Q.size(); j++){
                        if(W.at<uchar>(i,Q[j]) == 0 || i == Q[j]){
                            is_consistent_with_all_in_Q = false;
                            break;
                        }
                    }
                    if(is_consistent_with_all_in_Q == true){
                        compatible_matches.push_back(i);
                    }
                }

                best_match_score = 0.;
                for(size_t i=0; i<compatible_matches.size(); i++){

                    if(sumW.at<double>(0,compatible_matches[i]) > best_match_score){

                        best_match_score = sumW.at<double>(0, compatible_matches[i]);
                        best_match_idx = compatible_matches[i];
                    }
                }

                if(compatible_matches.size() != 0){
                    Q.push_back(best_match_idx);
                    inliers.push_back(matches[best_match_idx]);
                } else{
                    break;
                }
            }
            return inliers;
       }

// ------------------------------------------------------------------------------------------------------------------------

    Eigen::Isometry3d* HowardMotionEstimator::makeInitialMotionEstimate(std::vector<DMatch> consistent_matches){
          int num_inliers = consistent_matches.size();

          Eigen::Isometry3d* motion_estimate = new Eigen::Isometry3d();
          motion_estimate->setIdentity();

          if (consistent_matches.size() < 6) {
            return motion_estimate;
          }

          double wax, way, waz,
                    wbx, wby, wbz;

          int feat_idx_a, feat_idx_b;

          // gather all the inliers into two big matrices
          Eigen::MatrixXd target_xyz(3, num_inliers);
          Eigen::MatrixXd ref_xyz(3, num_inliers);

          for (size_t i = 0; i < consistent_matches.size(); i++) {
                feat_idx_a = consistent_matches[i].queryIdx;
                feat_idx_b = consistent_matches[i].trainIdx;

                cv::Vec3f coords_3d_a = this->leftFrameA->keypointsOnWorldCoordinates[feat_idx_a];
                cv::Vec3f coords_3d_b = this->leftFrameB->keypointsOnWorldCoordinates[feat_idx_b];

                wax = (double) coords_3d_a[0];
                way = (double) coords_3d_a[1];
                waz = (double) coords_3d_a[2];

                wbx = (double) coords_3d_b[0];
                wby = (double) coords_3d_b[1];
                wbz = (double) coords_3d_b[2];

                ref_xyz(0,i) = wax;
                ref_xyz(1,i) = way;
                ref_xyz(2,i) = waz;

                target_xyz(0,i) = wbx;
                target_xyz(1,i) = wby;
                target_xyz(2,i) = wbz;
          }

        // target = y_i; ref = x_i
        // estimate R and T such that (1/n)*sum(|y_i - (R x_i + T)|^2) is minimized
        Eigen::Matrix4d umeyama_est = Eigen::umeyama(ref_xyz, target_xyz, false);
        *motion_estimate = Eigen::Isometry3d(umeyama_est);

        return motion_estimate;
}

// ------------------------------------------------------------------------------------------------------------------------


double *HowardMotionEstimator::refineMotionEstimate(std::vector<DMatch> inliers, double *initial_params){

  	const size_t num_inliers = inliers.size();
	const size_t num_params = 7; // [qw qx qy qz tx ty tz]

	if(num_inliers < num_params){
        // cout << "insufficient number of inliers" << endl;
        return initial_params;
	}

	cv::Mat *P_left = this->leftRectifiedProjectionMatrix;
	const double p11 = P_left->at<double>(0,0), p12 = P_left->at<double>(0,1), p13 = P_left->at<double>(0,2), p14 = P_left->at<double>(0,3),
				 p21 = P_left->at<double>(1,0), p22 = P_left->at<double>(1,1), p23 = P_left->at<double>(1,2), p24 = P_left->at<double>(1,3),
				 p31 = P_left->at<double>(2,0), p32 = P_left->at<double>(2,1), p33 = P_left->at<double>(2,2), p34 = P_left->at<double>(2,3);

	// allocate memory for the arrays inside the Data struct
	double jax_data[num_inliers];
	double jay_data[num_inliers];
	double jbx_data[num_inliers];
	double jby_data[num_inliers];
	double wax_data[num_inliers];
	double way_data[num_inliers];
	double waz_data[num_inliers];
	double wbx_data[num_inliers];
	double wby_data[num_inliers];
	double wbz_data[num_inliers];
	bool   is_outlier_data[num_inliers];
	double cam_mat_data[] = {p11, p12, p13, p14, p21, p22, p23, p24, p31, p32, p33, p34};

  	// wrap data into a Data struct
	Data data = {num_inliers, jax_data, jay_data, jbx_data, jby_data,
                        wax_data, way_data, waz_data, wbx_data, wby_data, wbz_data,
                                cam_mat_data, is_outlier_data};

	double jax, jay, jbx, jby,
				wax, way, waz, wbx, wby, wbz;

    for(size_t i=0; i<num_inliers; i++){

    	int feat_idx_a = inliers[i].queryIdx;
    	int feat_idx_b = inliers[i].trainIdx;

        cv::KeyPoint kp_a = this->leftFrameA->keypointsOnImageCoordinates[feat_idx_a];
		cv::KeyPoint kp_b = this->leftFrameB->keypointsOnImageCoordinates[feat_idx_b];

		cv::Vec3f coords_3d_a = this->leftFrameA->keypointsOnWorldCoordinates[feat_idx_a];
		cv::Vec3f coords_3d_b = this->leftFrameB->keypointsOnWorldCoordinates[feat_idx_b];

		jax = kp_a.pt.x;
		jay = kp_a.pt.y;

		jbx = kp_b.pt.x;
		jby = kp_b.pt.y;

		wax = (double) coords_3d_a[0];
		way = (double) coords_3d_a[1];
		waz = (double) coords_3d_a[2];

		wbx = (double) coords_3d_b[0];
		wby = (double) coords_3d_b[1];
		wbz = (double) coords_3d_b[2];

		data.jax_data[i] = jax;	data.jay_data[i] = jay;
		data.jbx_data[i] = jbx;	data.jby_data[i] = jby;

		data.wax_data[i] = wax;	data.way_data[i] = way;	data.waz_data[i] = waz;
		data.wbx_data[i] = wbx;	data.wby_data[i] = wby;	data.wbz_data[i] = wbz;
	}

	// get initial estimate obtained by Umeyama's (or any other) method
	double q[] = {initial_params[0], initial_params[1], initial_params[2], initial_params[3]};
	double t[] = {initial_params[4], initial_params[5], initial_params[6]};

	// perform the initial filtering of remaining outliers, considering only the inital estimation, as described in the paper
	double error_thresh = 0.1;
	int matches_removed = 0;
	for(size_t i=1; i<num_inliers; i++){
		double ja[] = {jax_data[i], jay_data[i]};
		double jb[] = {jbx_data[i], jby_data[i]};
		double wa[] = {wax_data[i], way_data[i], waz_data[i]};
		double wb[] = {wbx_data[i], wby_data[i], wbz_data[i]};

		double error = bidirectional_reprojection_error(ja, jb, wa, wb, q, t, cam_mat_data);
		if(error > error_thresh){
			is_outlier_data[i] = true;
			matches_removed++;
		}else{
			is_outlier_data[i] = false;
        }
	}

	// initial_params = {qw, qx, qy, qz, tx, ty, tz};
	gsl_multifit_fdfsolver * solver = run_optimization(num_params, 2*num_inliers, &data, initial_params);
	double qw = gsl_vector_get (solver->x, 0);
	double qx = gsl_vector_get (solver->x, 1);
	double qy = gsl_vector_get (solver->x, 2);
	double qz = gsl_vector_get (solver->x, 3);
	double tx = gsl_vector_get (solver->x, 4);
	double ty = gsl_vector_get (solver->x, 5);
	double tz = gsl_vector_get (solver->x, 6);

	double quat_norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= quat_norm;
	qx /= quat_norm;
	qy /= quat_norm;
	qz /= quat_norm;

	double* optimized_params = new double[7];
	optimized_params[0] = qw;
	optimized_params[1] = qx;
	optimized_params[2] = qy;
	optimized_params[3] = qz;
	optimized_params[4] = tx;
	optimized_params[5] = ty;
	optimized_params[6] = tz;

	return optimized_params;
}

// ------------------------------------------------------------------------------------------------------------------------

        HowardMotionEstimator::HowardMotionEstimator(DepthSource *depthSource, cv::Mat *leftRectProjectionMat, AbstractFeatureDetector *detector, AbstractFeatureDescriptor *descriptor){
            this->depthSource = depthSource;
            this->leftRectifiedProjectionMatrix = leftRectProjectionMat;
            this->detector = detector;
            this->descriptor = descriptor;
            this->lastMotionEstimate = Eigen::Isometry3d();
            lastMotionEstimate.setIdentity();
	    }


         // Frames loaded must be undistorted and with epipolar lines horizontally aligned (distortion removal + stereo rectification)
        void HowardMotionEstimator::loadNewRectifiedStereoPair(std::shared_ptr<Frame> left, std::shared_ptr<Frame> right){
                double elapsed_time = 0;
                double tick0 = (double)cv::getTickCount();

                // Check if this is the first stereo pair being loaded..
                if(this->leftFrameA == nullptr && this->rightFrameA == nullptr && this->leftFrameB == nullptr && this->rightFrameB == nullptr){
                        this->detectValidFeatures(left, this->depthSource, this->detector);
                        this->descriptor->describeFeatures(left);
                        this->leftFrameB  = left;
                        this->rightFrameB = right;
                } else {
                        this->detectValidFeatures(left, this->depthSource, this->detector);
                        elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
                        cout << "time spent on feature detection: "<< 1/elapsed_time*1000 << " ms"<< endl;
                        tick0 = (double)cv::getTickCount();

                        this->descriptor->describeFeatures(left);
                        elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
                        cout << "time spent on feature description: "<< 1/elapsed_time*1000 << " ms"<< endl;
                        tick0 = (double)cv::getTickCount();

                        // shift the stereo pair, so the computations can be done only on frames at tB
                        this->leftFrameA  = this->leftFrameB;
                        this->rightFrameA = this->rightFrameB;
                        this->leftFrameB  = left;
                        this->rightFrameB = right;
                }
        }
// ------------------------------------------------------------------------------------------------------------------------

		void HowardMotionEstimator::estimateIncrementalMotion(){

                double elapsed_time = 0;

                if(this->leftFrameA==nullptr && this->rightFrameA==nullptr)
                    return;

                // match features between leftA and leftB
                double tick0 = (double)cv::getTickCount();

                std::vector<cv::DMatch> allMatches = FeatureMatchingUtils::matchWithCrossCheck(this->leftFrameA->descriptors.get(), this->leftFrameB->descriptors.get());

                elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
                cout << "time spent on feature matching: "<< 1/elapsed_time*1000 << " ms"<< endl;

                // build consistency matrix W and select consistent matches only
                tick0 = (double)cv::getTickCount();

                std::vector<DMatch> consistentMatches = this->selectConsistentMatches(allMatches, 0.05);

                elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
                cout << "time spent on inliers check: "<< 1/elapsed_time*1000 << " ms"<< endl;

                tick0 = (double)cv::getTickCount();

                Eigen::Isometry3d *initialMotionEstimate =  this->makeInitialMotionEstimate(consistentMatches);
                Eigen::Vector3d vecT  = (*initialMotionEstimate).translation();
                Eigen::Quaternion<double> vecRotQuat = Eigen::Quaternion<double>((*initialMotionEstimate).rotation());

                double t[] = {vecT[0], vecT[1], vecT[2]};
                double quat[] = {vecRotQuat.w(), vecRotQuat.x(), vecRotQuat.y(), vecRotQuat.z()};

                double initialParams[] = {quat[0], quat[1], quat[2], quat[3], t[0], t[1], t[2]};
                double *refinedParams = this->refineMotionEstimate(consistentMatches, initialParams);

                // build a Isometry3d with the refined parameters
                Eigen::Quaternion<double> refinedQuat(refinedParams[0], refinedParams[1], refinedParams[2], refinedParams[3]);
                Eigen::Matrix<double,3,1> refinedT(refinedParams[4], refinedParams[5], refinedParams[6]);

                Eigen::Isometry3d refinedMotionEstimate = Eigen::Isometry3d();
                refinedMotionEstimate.setIdentity();
                refinedMotionEstimate.rotate(refinedQuat);
                refinedMotionEstimate.translate(refinedT);

                elapsed_time =  1/((cv::getTickCount() - tick0) / cv::getTickFrequency());
                cout << "time spent on motion estimation: "<< 1/elapsed_time*1000 << " ms"<< endl;

                delete(initialMotionEstimate);

                this->lastMotionEstimate = refinedMotionEstimate;

                return;
		}


// ------------------------------------------------------------------------------------------------------------------------


		Eigen::Isometry3d HowardMotionEstimator::getIncrementalMotionEstimate(){
            return this->lastMotionEstimate;
		}


