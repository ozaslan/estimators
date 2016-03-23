#ifndef _OPTICAL_FLOW_ESTIMATOR_HH_
#define _OPTICAL_FLOW_ESTIMATOR_HH_

#include <opencv2/opencv.hpp>
#include <utils.hh>
 
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
  
using namespace std;

/*
  This class implements optical flow estimation algorithms.
  Depending on the paramters, the method can be grid-based or dense
  flow estimation. Input to the class is a stream of images. It also
  expects a configuration file path as well. It also clusters regions 
  of the image into homography classes using RANSAC. 

*/
class OpticalFlowEstimator{
public:
  struct OFEParams{
    int grid_cols;
    int grid_rows;
    int pyra_levels;
    cv::Size win_size;
    bool use_init_flow;
    int border;
  };
private:
  int _img_idx;
  cv::Mat _imgs[2];
  vector<cv::Mat> _pyras[2];
  OFEParams _params;

  vector<cv::Point2f> _tails;
  vector<cv::Point2f> _tips;

  vector<unsigned char> _status;
  vector<unsigned char> _cluster_ids;
  vector<float> _err;

  cv::TermCriteria _term_criteria;

  vector<cv::Scalar> _colors;

  int _calc_tail_coords();
  int _initialize();
public:
  // The constructors initialize the internal parameters and caches.
  // The constructor with the OFEParams can be used to initalize the
  // object with specific parameters. This can also be done using the
  // 'set_params(...)' function. If one of more of the parameters is
  // not permissible, the constructor prints an error message and sets
  // the corresponding paramters to default values.
  OpticalFlowEstimator();
  OpticalFlowEstimator(const OFEParams &params);
  // Through this function, the programmer registers new images to the
  // object. For performance concerns, OF estimation and related pre-processing
  // is postponed to 'estimate_of(...)' function call. This functions returns
  // '0' if successful and '-1' if the image size, type does not match
  // with the previous image. 
  int push_image(const cv::Mat &img);
  // This function assumes that the programmer pushed at least two images
  // before being called. It estimates the optical flow field depending on
  // the parameters defined using 'set_params(...)' function. The results
  // are stored internally which can be fetched using 'get_of_vectors(...)'.
  // This function returns '0' if successful, otherwise returns '-1'. The latter
  // happens when no or one image is pushed or the image properties such as
  // size, type do not match.
  int estimate_of();
  // This function returns the optical flow field vectors in a pair vector.
  // The first and second arguments  are tail and tip coordinates of the vectors 
  // respectively. If there is a least one successful vector, the return value
  // is '0', otherwise '-1'.
  int get_of_vectors(vector<cv::Point2f> &tails, vector<cv::Point2f> &tips);
  // This function can be used to update the paramters. If one or more of the
  // parameters is not within the permissible values, this function returns
  // '-1' and leaves the corresponding parameters with their current values, otherwise 
  // returns '0'.
  int set_params(const OFEParams &params);
  OFEParams get_params(){ return _params;}
  // This function plots the optical flow field on the given image.
  // If the image is not of the original image size, internally stores copy
  // of the previous frame is cloned to 'image'. If flow field has not been
  // estimated yet, this function returns '-1' otherwise '0'.
  int plot_of_field(cv::Mat &image);
  // This functions clusters the OF vectors into homography groups
  // using sample consensus algorithms like RANSAC, LMED.
  int cluster_planes();
  // This function returns the cluster ids determined by the 'cluster_planes(...)'
  // function. This returns '0' is the '_cluster_ids' vector is non-empty, otherwise
  // '-1'.
  int get_cluster_ids(vector<unsigned char> &cluster_ids){
    cluster_ids = _cluster_ids;
    return _cluster_ids.size() == 0 ? -1 : 0;
  }
};

#endif
