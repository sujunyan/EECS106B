#include <utils.hpp>

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


pcl::PointXYZRGB findCenter(const PointCloud::ConstPtr& msg)
{
	int col = msg->width / 2; 
	int row = msg->height / 2;

	pcl::PointXYZRGB track_point;

  	
	track_point.x = msg->at(col, row).x;
	track_point.y = msg->at(col, row).y;
	track_point.z = msg->at(col, row).z;
	
    return track_point;
}