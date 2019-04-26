#include <ros/ros.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include "sensor_msgs/PointCloud2.h"
#include <math.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

pcl::PointXYZRGB findCenter(const PointCloud::ConstPtr& msg);

float get2Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);
float get3Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);
float concavityOfPair(pcl::PointXYZRGB& a, pcl::PointXYZRGB& b, pcl::Normal& an, pcl::Normal bn);
float pDist(pcl::PointXYZRGB& a, pcl::PointXYZRGB& b);
float pDistp(pcl::PointXYZ& a, pcl::PointXYZ& b);
float surfaceElementArea(pcl::PointXYZRGB& o, pcl::PointXYZRGB& a1, pcl::PointXYZRGB& a2, pcl::PointXYZRGB& b1, pcl::PointXYZRGB& b2);
