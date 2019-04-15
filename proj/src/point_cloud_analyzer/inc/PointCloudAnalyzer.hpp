#include <ros/ros.h>

class PointCloudAnalyzer
{
private:
    /* data */
public:
    PointCloudAnalyzer(/* args */);
    ~PointCloudAnalyzer();
private:
    ros::Publisher  m_pubGray;

};


