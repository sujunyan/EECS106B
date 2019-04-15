#include "PointCloudAnalyzer.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

PointCloudAnalyzer::PointCloudAnalyzer(/* args */)
{
}

PointCloudAnalyzer::~PointCloudAnalyzer()
{
    setup_parameter();
}

void PointCloudAnalyzer::setup_parameter(){
    use_case_pub = nh.advertise<std_msgs::String>("/use_case", 1);
    max_filter_pub = nh.advertise<std_msgs::Float32>("/max_filter", 1);
    min_filter_pub = nh.advertise<std_msgs::Float32>("/min_filter", 1);
    exposure_pub = nh.advertise<std_msgs::UInt32>("/expo_time", 1);
	sleep(3);

	std_msgs::String mode_msg("MODE_5_45FPS_500");
	use_case_pub.publish(mode_msg);

    std_msgs::Float32 max_filter_msg;
	max_filter_msg.data = 0.3;
	max_filter_pub.publish(max_filter_msg);

	std_msgs::Float32 min_filter_msg;
	min_filter_msg.data = 0.06;
	min_filter_pub.publish(min_filter_msg);

	std_msgs::UInt32 expo_time_msg;
	expo_time_msg.data = 50;
	exposure_pub.publish(expo_time_msg);
	sleep(3);

	ros::spinOnce();	
}

void PointCloundAnalyzer::start(){
	point_cloud_sub = nh.subscribe<PointCloud>("/royale_camera_driver/point_cloud", 1, callback);
}

void PointCloudAnalyzer::callback(const PointCloud::ConstPtr& msg)
{

}

