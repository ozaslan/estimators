#include "of_estimator.hh"

int OpticalFlowEstimator::_initialize(){
	_params.grid_cols = 29;
	_params.grid_rows = 21;
	_params.pyra_levels = 3;
	_params.win_size = cv::Size(21, 21);
	_params.use_init_flow = true;
	_params.border = 30;
	_img_idx = 0;

	_term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

	vector<Eigen::Vector3i> temp_colors;
	utils::generate_colors(temp_colors);
	_colors.resize(temp_colors.size());
	for(int i = 0 ; i < (int)temp_colors.size() ; i++){
		_colors[i](0) = temp_colors[i](0);
		_colors[i](1) = temp_colors[i](1);
		_colors[i](2) = temp_colors[i](2);
	}

	_tails.clear();
	_tips.clear();

	return 0;
}

int OpticalFlowEstimator::_calc_tail_coords(){

	if(_tails.size() != 0)
		return 0;
	
	if(_pyras[0].size() == 0 || 
	   _pyras[0].size() != _pyras[1].size()){
		_tails.clear();
		_tips.clear();
		return -1;
	}	

	cv::Size img_size = _pyras[0][0].size();
	int eff_width, eff_height; // effective width and height

	if((eff_width  = img_size.width  - 2 * _params.border) <= 0 ||
	   (eff_height = img_size.height - 2 * _params.border) <= 0){
		_tails.clear();
		_tips.clear();
		return -1;
	}

	double dr = eff_height / _params.grid_rows;
	double dw = eff_width  / _params.grid_cols;

	_tails.reserve(_params.grid_cols * _params.grid_cols);
	 _tips.reserve(_params.grid_cols * _params.grid_cols);

	for(double r = 0 ; r <= eff_height ; r += dr)
		for(double c = 0 ; c <= eff_width ; c += dw)
			_tails.push_back(cv::Point2f(c + _params.border, r + _params.border));

	return 0;
}

OpticalFlowEstimator::OpticalFlowEstimator(){
	_initialize();
}

OpticalFlowEstimator::OpticalFlowEstimator(const OFEParams &params){
	_initialize();
	set_params(params);
}

int OpticalFlowEstimator::push_image(const cv::Mat &img){
	ASSERT(img.channels() == 1, "Input image has to be gray scale")

	cv::buildOpticalFlowPyramid(img, _pyras[_img_idx], _params.win_size, _params.pyra_levels);
	_img_idx = (_img_idx + 1) % 2;

	if(_pyras[0].size() != _pyras[1].size() ||
	   _pyras[0][0].size() != _pyras[1][0].size() ||
	   _pyras[0][0].depth() != _pyras[1][0].depth() ||
	   _pyras[0][0].channels() != _pyras[1][0].channels()){
		_tails.clear();
		return -1;
	} else {
		_calc_tail_coords();
		return 0;
	}
}

int OpticalFlowEstimator::estimate_of(){
	if(_tails.size() == 0)
		return -1;

	if(_params.use_init_flow &&  _tails.size() == _tips.size()){
		for(int i = 0 ; i < (int)_tails.size() ; i++){
			if(_status[i] == 0)
				_tips[i] = _tails[i];
		}
	}

	cv::calcOpticalFlowPyrLK(_pyras[(_img_idx + 1) % 2], _pyras[_img_idx], _tails, _tips, _status, _err, _params.win_size, _params.pyra_levels, _term_criteria, 
							(_params.use_init_flow && _tails.size() == _tips.size() ? cv::OPTFLOW_USE_INITIAL_FLOW : 0) + cv::OPTFLOW_LK_GET_MIN_EIGENVALS);

	return 0;
}

int OpticalFlowEstimator::get_of_vectors(vector<cv::Point2f> &tails, vector<cv::Point2f> &tips){
	tails = _tails;
	tips  = _tips;
	return _tails.size() == 0 ? -1 : 0;
}

int OpticalFlowEstimator::set_params(const OFEParams &params){
	
	int retval = 0;
	if(params.grid_cols >= 1 && params.grid_cols <= 1000)
		_params.grid_cols = params.grid_cols;
	else
		retval = -1;
	if(params.grid_rows >= 1 && params.grid_rows <= 1000)
		_params.grid_rows = params.grid_rows;
	else
		retval = -1;
	if(params.pyra_levels >= 1 && params.pyra_levels < 32)
		_params.pyra_levels = params.pyra_levels;
	else
		retval = -1;
	if(params.win_size.width  >= 7 && params.win_size.width <= 100 &&
	   params.win_size.height >= 7 && params.win_size.height <= 100)
		_params.win_size = params.win_size;
	else
		retval = -1;
	_params.use_init_flow = params.use_init_flow;
	
	if(params.border >= 0 && params.border <= 100)
		_params.border = params.border;
	else
		retval = -1;
	return retval;
}

int OpticalFlowEstimator::plot_of_field(cv::Mat &image){
	if(_tails.size() == 0)
		return -1;
	if(image.size() != _pyras[(_img_idx + 1) % 2][0].size())
		_pyras[(_img_idx + 1) % 2][0].copyTo(image);

	cv::Scalar color(0, 250, 0);

	int num_vectors = _tails.size();
	for(int i = 0 ; i < num_vectors ; i++){
		if(_status[i] != 0){
			if(_cluster_ids.size() != 0)
				color = _colors[_cluster_ids[i]];
			cv::Point2d tip = _tails[i] + 3 * (_tips[i] - _tails[i]);
			cv::line(image, _tails[i], tip, color, 1);
			//cv::line(image, _tails[i], _tips[i], cv::Scalar(0, 250, 0), 1);
			//cv::arrowedLine(image, _tails[i], _tips[i], cv::Scalar(0, 255, 0), 1, cv::CV_AA, 0, 0.1);
			cv::circle(image, _tails[i], 2.3, color, -1);
		}
	}

	return 0;
}


cv::Mat OpticalFlowEstimator::cluster_planes(){
	if(_tails.size() == 0 || _tips.size() != _tails.size()){
		_cluster_ids.clear();
		return -1;
	}

	//++: Mat findHomography(InputArray srcPoints, InputArray dstPoints, int method=0, double ransacReprojThreshold=3, OutputArray mask=noArray() )¶
	return cv::findHomography(_tails, _tips, cv::RANSAC, 1.3, _cluster_ids);
}
