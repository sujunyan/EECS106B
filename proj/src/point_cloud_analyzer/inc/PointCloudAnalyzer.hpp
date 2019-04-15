#include <ros/ros.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
class PointCloudAnalyzer
{
private:
    /* data */
public:
    PointCloudAnalyzer(/* args */);
    ~PointCloudAnalyzer();
    void start();
private:
    void setup_parameter();
    void setup_vis();
    static void callback(const PointCloud::ConstPtr& msg);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    ros::NodeHandle nh;
    ros::Publisher use_case_pub;
	ros::Publisher max_filter_pub;
	ros::Publisher min_filter_pub;
	ros::Publisher exposure_pub;
    
    ros::Subscriber point_cloud_sub;


};


