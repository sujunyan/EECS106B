#include <ros/ros.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include "sensor_msgs/PointCloud2.h"


#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>    
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <utils.hpp>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class PointCloudAnalyzer
{
private:
    /* data */
    PointCloud::Ptr full_membrane;
    PointCloud::Ptr deformed_membrane;
    PointCloud::Ptr contact_membrane;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    
    pcl::PointXYZRGB center_point;
    pcl::PointXYZRGB calibrate_point; 

public:
    PointCloudAnalyzer(/* args */);
    ~PointCloudAnalyzer();
    void start();
private:
    void setup_parameter();
    void setup_vis();
    void callback(const PointCloud::ConstPtr& msg);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    
    ros::NodeHandle nh;
    ros::Publisher use_case_pub;
	ros::Publisher max_filter_pub;
	ros::Publisher min_filter_pub;
	ros::Publisher exposure_pub;
    
    ros::Subscriber point_cloud_sub;

    PointCloud::Ptr Genfullmem(const PointCloud::ConstPtr& msg);
    PointCloud::Ptr Gendeformem(const PointCloud::Ptr& msg);
    PointCloud::Ptr Gencontact(const PointCloud::Ptr& msg);
    pcl::PointCloud<pcl::Normal>::Ptr Getnormal(PointCloud::Ptr& msg);

    //color of area      green for deformation   white for membrane
    uint8_t r_green = 113;
    uint8_t g_green = 164;
    uint8_t b_green = 252;
    uint32_t green = ((uint32_t) r_green << 16 | (uint32_t)g_green << 8 | (uint32_t)b_green);

    uint8_t r_white = 206;
    uint8_t g_white = 206;
    uint8_t b_white = 206;
    uint32_t white = ((uint32_t) r_white << 16 | (uint32_t)g_white << 8 | (uint32_t)b_white);

    uint8_t r_red =  252;
    uint8_t g_red = 196;
    uint8_t b_red = 113;
    uint32_t red = ((uint32_t) r_red << 16 | (uint32_t)g_red << 8 | (uint32_t)b_red);
    
    
    
    
    //pcl::PointXYZRGB track_point;
};


