#include "of_estimator.hh"
#include "of_estimator/OpticalFlowField.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

/*
	-- Add IMU listener to improve tracking performace for fast motion
	-- Add option to publish undistorted flow field
*/

ros::Subscriber image_subs;
ros::Publisher  of_image_publ;
ros::Publisher  of_field_publ;

cv_bridge::CvImagePtr image_msg;
of_estimator::OpticalFlowField of_field_msg;

void process_inputs(const ros::NodeHandle &n);
int  setup_messaging_interface(ros::NodeHandle &n);
void image_callback(const sensor_msgs::Image &msg);
int  publish_of_field();
int  publish_of_image();

bool debug_mode;
OpticalFlowEstimator ofe;
OpticalFlowEstimator::OFEParams ofe_params;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "of_estimator_node");
	ros::NodeHandle n("~");

	process_inputs(n);	
	setup_messaging_interface(n);
	
	ros::spin();
	
	return 0;
}

void process_inputs(const ros::NodeHandle &n)
{
	n.param("debug_mode", debug_mode, false);

	n.param("grid_cols"		  , ofe_params.grid_cols, 29);
	n.param("grid_rows"		  , ofe_params.grid_rows, 21);
	n.param("pyra_levels"     , ofe_params.pyra_levels, 3);
	n.param("win_size_width"  , ofe_params.win_size.width , 21);
	n.param("win_size_height" , ofe_params.win_size.height, 21);
	n.param("use_init_flow"   , ofe_params.use_init_flow, true);
	n.param("border_thickness", ofe_params.border, 30);

	ROS_INFO(" ---------- OF ESTIMATOR NODE ------------");
	ROS_INFO("[debug_mode] ---------- : [%s]"      , debug_mode ? "TRUE" : "FALSE");
	ROS_INFO("grid_[rows, cols] ----- : [%d, %d]"  , ofe_params.grid_rows, ofe_params.grid_cols);
	ROS_INFO("[pyra_levels] --------- : [%d]"      , ofe_params.pyra_levels);
	ROS_INFO("[win_size](w-by-h) ---- : [%d-by-%d]", ofe_params.win_size.width, ofe_params.win_size.height);
	ROS_INFO("[use_init_flow] ------- : [%s]"      ,  ofe_params.use_init_flow ? "TRUE" : "FALSE");
	ROS_INFO("[border_thickness] ---- : [%d]"      , ofe_params.border);
	ROS_INFO(" -----------------------------------------");
	
	ofe.set_params(ofe_params);
}

int setup_messaging_interface(ros::NodeHandle &n)
{
	if(debug_mode)	{
		ROS_INFO("OF ESTIMATOR NODE : setting up messaging interface.");
		ROS_INFO(" --- Listening  : ~image_raw");
		ROS_INFO(" --- Publishing : ~of_field");
		ROS_INFO(" --- Publishing : ~of_image");
	}
		
	image_subs  = n.subscribe("image_raw", 10, image_callback, ros::TransportHints().tcpNoDelay());
	of_field_publ  = n.advertise<of_estimator::OpticalFlowField>("of_field", 10);
	of_image_publ  = n.advertise<sensor_msgs::Image>("of_image", 10);

	return 0;
}

int publish_of_field()
{
	if(debug_mode)
		ROS_INFO("OF ESTIMATOR NODE : Published of OF field to ~of_field");

	of_field_msg.header.seq++;
	of_field_msg.header.stamp = ros::Time::now();
	of_field_msg.header.frame_id = "camera";

	static vector<cv::Point2f> tails, tips;
	ofe.get_of_vectors(tails, tips);

	of_field_msg.num_vectors = tails.size();
	of_field_msg.tails.resize(of_field_msg.num_vectors);
	of_field_msg.tips.resize(of_field_msg.num_vectors);
	for(unsigned int i = 0 ; i < of_field_msg.num_vectors ; i++){
		of_field_msg.tails[i].x = tails[i].x;
		of_field_msg.tails[i].y = tails[i].y;
		of_field_msg.tails[i].z = 0;
		of_field_msg.tips[i].x = tips[i].x;
		of_field_msg.tips[i].y = tips[i].y;
		of_field_msg.tips[i].z = 0;
	}

	of_field_publ.publish(of_field_msg);

	return 0;
}

int publish_of_image()
{
	if(debug_mode)
		ROS_INFO("OF ESTIMATOR NODE : Published of OF image to ~of_image");

	ofe.plot_of_field(image_msg->image);
	of_image_publ.publish(image_msg->toImageMsg());

	return 0;
}

void image_callback(const sensor_msgs::Image &msg)
{
	if(debug_mode)
		ROS_INFO("OF ESTIMATOR NODE : Got Image message!");	

	try	{
		image_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	static cv::Mat img;
	if(image_msg->image.channels() == 3)
		cv::cvtColor(image_msg->image, img, CV_BGR2GRAY);
	else
		img = image_msg->image;

	ofe.push_image(img);
	ofe.estimate_of();

	publish_of_field();
	publish_of_image();
}


