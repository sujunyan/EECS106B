#include "PointCloudAnalyzer.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>	
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
//#include <pcl/filters/voxel_grid.h>

PointCloudAnalyzer::PointCloudAnalyzer(/* args */)
{
	setup_parameter();
	setup_vis();
}

PointCloudAnalyzer::~PointCloudAnalyzer()
{
}

void PointCloudAnalyzer::setup_parameter(){
	std::cout<<"setting up parameters \n";
    use_case_pub = nh.advertise<std_msgs::String>("/use_case", 1);
    max_filter_pub = nh.advertise<std_msgs::Float32>("/max_filter", 1);
    min_filter_pub = nh.advertise<std_msgs::Float32>("/min_filter", 1);
    exposure_pub = nh.advertise<std_msgs::UInt32>("/expo_time", 1);
	sleep(5);

	std_msgs::String mode_msg;
	mode_msg.data = "MODE_5_45FPS_500";
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


void PointCloudAnalyzer::setup_vis(){

	std::cout<<"setting up vis \n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Creating point cloud\n\n";

	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	point_cloud_ptr->height = 1;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer = viewer0;
	viewer->setBackgroundColor (255, 255, 255);
	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "deformed");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "deformed");

	// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_JET, "cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->removeCoordinateSystem();

	std::cout << "Created point cloud\n";
}

void PointCloudAnalyzer::callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	std::cout << "PointCloud message recieved\n";

	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*pcl_msg);

	printf ("Cloud: width = %d, height = %d\n", pcl_msg->width, pcl_msg->height);
	viewer->updatePointCloud(pcl_msg, "cloud");
	viewer->updatePointCloud(pcl_msg, "deformed");
}



void PointCloudAnalyzer::start(){
	point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/royale_camera_driver/point_cloud", 1, &PointCloudAnalyzer::callback, this);
	//point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/royale_camera_driver/point_cloud", 10, callback );
	ros::Rate r(10); // 10 hz
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		ros::spinOnce();
		r.sleep(); // TODO might has a bug
	}
}



