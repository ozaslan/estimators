#ifndef __VISIO_BASED_TUNNEL_LOCALIZER_HH__
#define __VISIO_BASED_TUNNEL_LOCALIZER_HH__

#include <list>
#include <vector>
#include <string>
#include <numeric>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/octree/octree.h>

#include <utils.hh>
#include <uniform_feat_tracker.hh>
#include <uniform_feat_extractor.hh>

#include "camera_calib_params.hh"

using namespace std;

/*

   This class is an implementation of vision based pose estimator.
   Thus by definition, it gets a number of inputs, the map and initial
   estimates and outputs a pose estimate. It also provides an uncertainty
   for its estimate. 

   The inputs to the class are image sequences. Images can be from N different
   sources. Consider an image sequence from one source. This class initializes
   and tracts a number of features along the image sequence. By utilizing the 
   given map, initial pose estimates (provided with every grabbed frame) and 
   optical flow vectors, it tries to estimate the displacement of the robot 
   along the tunnel axis. For convenience this direction is by-definition
   aligned with world-x direction. This changes as the robot flies through
   different segments of the tunnel.

   In other words, it tracks features, does ray casting on to the map with
   the origin as the pose estimate provided by the 'RangeBasedTunnelLocalizer'.
   Then extract the x displacement.
   */

class VisionBasedTunnelLocalizer{
	private:
		vector<UniformFeatureTracker>	_trackers;  // set of feature trackers for each camera
		UniformFeatureExtractor			_extractor;	// is the feature tracker use by all trackers. 
		Eigen::Matrix4d					_poses[2];	// poses from RangeBasedTunnelLocalizer. 
		//  These store the previous pose at which the last 
		//  estimation was carried and the current pose.
		vector<CameraCalibParams>		_cam_params;	//  
		vector<Eigen::Matrix3d>	_inv_camera_matrices;	// inverter camera matrices for speed-up

		int _num_frames_pushed;	// counts the number of calls to 'push_image_data(...)' after the last 'estimate_displacement(...)' function call.

		// These vectors are filled by UniformFeatureTracker::get_features(...). Since that function
		// allocates memory, frequent calls should be avoided. 'Tip' of the flow vector
		// becomes the 'tail' in the next 'estimate_displacement(...)' call. For this reason
		// flow tips and tails are stored in a cyclic buffer.
		int _feat_buffer_idx;						// defines the buffer to be used in the current estimation step.
		vector<vector<cv::Point2f> > _feats[2];		// features as tracked by the UniformFeatureTracker.
		vector<vector<int> >		 _feat_ids[2];	// feature ids returned by UniformFeatureTracker::get_features(...)

		// Feature tracker and extractor related parameters.
		int _num_feats;			// total # of features to be tracked by trackers.
		int _of_history_len;	// length of the optical flow trailers.
		double _feat_threshold; // feature detector feature quality threshold.
		string _detector_type;  // type of the feature extractor which is one of "FAST", "GFTT" or "Harris"

		// Used to do ray casting to find corresponding map points to an image feature.
		pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *_octree;
		vector<pcl::PointXYZ> _of_tail_vecs, _of_tip_vecs;	// back-projected optical flow vectors.

		// Estimation results
		double _x_disp;			// displacement estimate along tunnel axis direction (world-x dir.).
		double _x_var;			// variance of '_x_disp'
		Eigen::Matrix6d _fim;	// Fisher information matrix ### will I use this?

		// ### Maybe I should check epipolar geometry. After seeing how outlier
		// elimination works in UniformFeatureTracker.

		// This function clears the history of the feature tracking, previous stored frames
		// as well as the calibration parameters. Call to this function is not required 
		// but might be useful when the user thinks that the feature tracking history is not quality.
		bool _reset();
	public:
		// 
		VisionBasedTunnelLocalizer(int num_feats = 100, int history_len = 7, 
				string detector_type = string("FAST"), 
				double threshold = 5);
		// This function calculates the displacement along the tunnel axis (world-x dir.) by 
		// utilizing the set of frames. At every call image features are extracted and
		// tracked from the previous frame. The user might want to estimate the displacement
		// after each frame vector insertion or wait indefinitely. However long intervals
		// may cause losing track of most of the features. This function incorporates 
		// the partial state estimate from RangeBasedTunnelLocalizer which is given 
		// through the 'pose' variable. This function assumes that each camera's relative 
		// position w.r.t. the platform is already given through 'register_params(...)'. 
		// Otherwise the function will raise an exception. And the order of the frames is 
		// assumed to be given in the same order at every call. This implicitly assumes 
		// that each frame maps to the same parameter throughout the lifetime of the class or 
		// until another parameter set is registered.
		bool push_camera_data(const vector<cv::Mat> &frames, const Eigen::Matrix4d &pose);
		// This function registers camera calibration parameters. Everytime a new parameter
		// set is registered, '_reset(...)' is called internally. The order of the frames supplied
		// to 'push_image_data(...)' and 'params' is assumed to be the same throughout the lifetime
		// of the object. Estimation and data pushing will not work unless proper number
		// of camera calibration parameters are give ahead of time. 
		bool register_camera_params(const vector<CameraCalibParams> &params);
		// ### to be implemented
		bool set_octree(pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree);
		// This function updates the octree with the given map. Thus frequent calls
		// might consume significant CPU time. The octree is used for ray-casting
		// of feature points.
		bool set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr &map);
		// This function gets the optical flow trailers, does ray-casting and geometrically 
		// estimates the displacement along the tunnel axis (world-x dir.).
		bool estimate_displacement(double &x_disp);
		// These functions assume that 'estimate_displacement(...)' function 
		// is called beforehand. When properly called, they return the displacement and the variance
		// along the tunnel axis (world-x dir.). Otherwise last displacement obtained at the most 
		// recent estimate is returned. The output of this function includes only 
		// the x-displacement and it is users responsibility to merge
		// with RangeBasedTunnelLocalizer's estimate.
		bool get_displacement(double &x_disp);
		bool get_variance(double &x_var);
		// This function gives the optical flow vectors back-projected to the
		// map. It uses the results of the 'estimate_displacement(...)' function. 
		// Hence it assumes that the estimator function has been already called. 
		// Otherwise flow vectors from the most recent estimation is returned 
		// without notification. Output of this function can be used for 
		// visualization or further external calculation regarding the estimator 
		// accuracy.
		bool get_back_projected_flow_vectors(vector<pcl::PointXYZ> &tails, vector<pcl::PointXYZ> &tips);
		// This functions estimates the covariance of the displacement 
		// estimate. It should be emphasized that the uncertainty regarding
		// the x-coordinate is for displacement, not the position. Also 
		// the off-diagonal elements encode the uncertainty cross-corelation 
		// between the estimated x-disp. and the given y-z and yaw estimates.
		// Thus the 6-by-6 covariance matrix has 1+2*3 = 7 non-zero element
		// (1 for x-disp, 3 for cross-corelation between x and y-z-yaw).
		bool get_covariance(Eigen::Matrix6d &cov);
		// This function plots the flow field starting from the latest displacement
		// estimation. If '_num_pushes <= _history_len' && 'plot_details = true', 
		// UniformFeatureTracker's detailed  trailer is used. Otherwise lenght-1 trailers
		// are plotted. This function also assumes that the 'img' vector is of same 
		// size with 'CameraCalibParams'. Otherwise it returns 'false' without any plots.
		bool plot_flows(vector<cv::Mat> &img, bool plot_flow, bool plot_feat); //###
};










#endif
