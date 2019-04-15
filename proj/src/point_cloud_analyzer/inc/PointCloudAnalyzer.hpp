#include <ros/ros.h>

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
    void callback(const PointCloud::ConstPtr& msg)

    ros::NodeHandle nh;
    ros::Publisher use_case_pub;
	ros::Publisher max_filter_pub;
	ros::Publisher min_filter_pub;
	ros::Publisher exposure_pub;
    
    ros::Subscriber point_cloud_sub;


};


