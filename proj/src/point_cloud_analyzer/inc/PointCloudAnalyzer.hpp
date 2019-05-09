#include <ros/ros.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>    
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/octree/octree.h>
#include <utils.hpp>
#include <queue>
#include <vector>

/*index =  r* msg->width + c  */
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
using namespace std;
class PointCloudAnalyzer
{
private:
    /* data */
    PointCloud::Ptr origin_membrane;
    PointCloud::Ptr origin_ptr;
    PointCloud::Ptr full_membrane;
    PointCloud::Ptr deformed_membrane;
    PointCloud::Ptr contact_membrane;
    PointCloud::Ptr map_ptr;
    PointCloud::Ptr transformed_memb_ptr;
    //PointCloud::Ptr test_smooth;
    //PointCloud::Ptr test_stat;
    PointCloud::Ptr filtered;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    

    pcl::PointXYZRGB center_point;
    pcl::PointXYZRGB calibrate_point; 
    pcl::PointXYZRGB origin_center_point;
    float transform_x,transform_y,transform_z;
    float center_z_depth;
    float origin_membrane_area;
    float full_membrane_area = 0;
    float contact_membrane_area = 0;
    float median_maxallowed_move = 0.01;
    int num_deform = 0;
    int num_concave = 0;
    int num_plane = 0;
    double rate = 0.4;
    double max_dis = 0; // store the max relative depth in real time
    double mean_dis = 0;
    double resolution = 0.01;        // 分辨率
    double stiffness = 0; // estimated stiffness
    vector<int> detectedPoints;
    double common_z,min_z;
    //filter size


    bool is_contact;
    
public:
    PointCloudAnalyzer(/* args */);
    ~PointCloudAnalyzer();
    void start();
private:
    void setup_parameter();
    void setup_vis();
    void callback(const PointCloud::ConstPtr& msg);
    // a sub-routine to calculate the max/mean relative depth or more information
    void depthCallback(const PointCloud::ConstPtr& msg); 
    // a sub-routine to estimate the stiffness 
    void stiffnessCallback(); 
    void checkContact(); // check if the membrane is contact to an object, will set the bool variable is_contact
    void armpos_callback(const geometry_msgs::Point::ConstPtr& msg);
    void addcontact(const PointCloud::Ptr& msg, bool);
    bool addFlag();
    void publishAll();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
    pcl::MedianFilter<pcl::PointXYZRGB> Median;
    
    ros::NodeHandle nh;
    ros::Publisher use_case_pub;
	ros::Publisher max_filter_pub;
	ros::Publisher min_filter_pub;
	ros::Publisher exposure_pub;

    ros::Publisher pub_contact_area;
    ros::Publisher pub_full_area;
    ros::Publisher pub_maxdis;
    ros::Publisher pub_meandis;

    ros::Subscriber point_cloud_sub;
    ros::Subscriber arm_sub;
    
    PointCloud::Ptr genFullmem(const PointCloud::Ptr& msg);
    PointCloud::Ptr genDeformem(const PointCloud::Ptr& msg);
    PointCloud::Ptr genContact(const PointCloud::Ptr& msg);
    PointCloud::Ptr transform(const PointCloud::Ptr& msg, float x, float y, float z);
    PointCloud::Ptr medianFilter(const PointCloud::ConstPtr& msg);

    float getsurfacearea(const PointCloud::Ptr&msg);
    pcl::PointCloud<pcl::Normal>::Ptr Getnormal(PointCloud::Ptr& msg);

    //predefeind colors of area      green for deformation   white for membrane
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
    uint8_t r_plane =  0;
    uint8_t g_plane = 255;
    uint8_t b_plane = 0;
    uint32_t plane = ((uint32_t) r_plane << 16 | (uint32_t)g_plane << 8 | (uint32_t)b_plane);
};


// a simple self-defined filter to average the data along a duration
class SimpleFilter{
public:
    SimpleFilter(int s):size(s){};
    double filter(const double &); // give the data of current time, output the filtered data.
private:
    int size; // the size of the filter
    vector<double> datas; // a vector to store the input datas.
    double averageValue(); // return the average value of current datas
};
