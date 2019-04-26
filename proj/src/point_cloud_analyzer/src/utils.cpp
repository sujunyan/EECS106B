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


float get2Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b)
{
	return pow(a.x - b.x, 2) + pow(a.y - b.y, 2);
}

float get3Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b)
{
    return sqrt(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0) + pow(a.z - b.z, 2.0));
}


float concavityOfPair(pcl::PointXYZRGB& a, pcl::PointXYZRGB& b, pcl::Normal& an, pcl::Normal bn){
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;

	float dnx = an.normal_x - bn.normal_x;
	float dny = an.normal_y - bn.normal_y;
	float dnz = an.normal_z - bn.normal_z;

	float concavity = dx * dnx + dy * dny + dz * dnz;

	return concavity;
}

float pDist(pcl::PointXYZRGB& a, pcl::PointXYZRGB& b) {
	return sqrt(pow(a.x - b.x, 2.0) + pow(a.y- b.y, 2.0) + pow(a.z - b.z, 2.0));
}

float pDistp(pcl::PointXYZ& a, pcl::PointXYZ& b) {
	return sqrt(pow(a.x - b.x, 2.0) + pow(a.y- b.y, 2.0) + pow(a.z - b.z, 2.0));
}

float surfaceElementArea(pcl::PointXYZRGB& o, pcl::PointXYZRGB& a1, pcl::PointXYZRGB& a2, pcl::PointXYZRGB& b1, pcl::PointXYZRGB& b2) {
	if (pcl::isFinite(a1) && pcl::isFinite(a2) && pcl::isFinite(b1) && pcl::isFinite(b2))  {
		float width = 0.5 * (pDist(o, a1) + pDist(o, a2));
		float height = 0.5 * (pDist(o, b1) + pDist(o, b2));
		return width * height;
	}  

	return 0.0;

}