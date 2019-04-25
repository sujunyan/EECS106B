#include <ros/ros.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include "sensor_msgs/PointCloud2.h"
#include <math.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointXYZRGB findCenter(const PointCloud::ConstPtr& msg);