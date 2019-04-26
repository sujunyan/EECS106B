#include "PointCloudMap.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>	


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/search/impl/search.hpp>
#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>
#include <vector>




PointCloudMap::PointCloudMap(/* args */)
{
	
	setup_vis();
}

PointCloudMap::~PointCloudMap()
{
}



void PointCloudMap::setup_vis(){
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    point_cloud_ptr->width = 0 ;
    point_cloud_ptr->height = 1 ;

	std::cout<<"setting up vis"<<"\n";
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    viewer = viewer1;
    std::cout<<"setting up back";
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    //viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //std::cout<<"addPlane"<<"\n";
    //pcl::ModelCoefficients coeffs;
    //coeffs.values.push_back (0.0);
    //coeffs.values.push_back (0.0);
    //coeffs.values.push_back (1.0);
    //coeffs.values.push_back (0.0);

    //std::vector<double> origin(3,0);
    
    //viewer->addPlane (coeffs, origin[0],origin[1],origin[2], "plane");

    viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    viewer->addCoordinateSystem (0.1);
    //viewer->initCameraParameters ();
 
}

void PointCloudMap::callback(const PointCloud::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    full_membrane->width = msg->width;
    full_membrane->height = msg->height;
    full_membrane->resize(full_membrane->height*full_membrane->width);

    std::cout << "PointCloud message recieved\n";
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);

    for (int c = 0; c < msg->width; c++) {
        for (int r = 0; r < msg->height; r++) {
            if (pcl::isFinite(msg->at(c, r))) {
                full_membrane->at(c, r).x = msg->at(c, r).x;
                full_membrane->at(c, r).y = msg->at(c, r).y;
                full_membrane->at(c, r).z = msg->at(c, r).z;    
                full_membrane->at(c,r).rgb = *reinterpret_cast<float*>(&green);
            }
        }
    }




    viewer->updatePointCloud(full_membrane, "cloud");
}



void PointCloudMap::start(){


    point_cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/royale_camera_driver/point_cloud", 1, &PointCloudMap::callback, this);
	ros::Rate r(10);
	while (!viewer->wasStopped ()){
  {
    viewer->spinOnce (100);
    ros::spinOnce();
    r.sleep();
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
}
// --------------
// -----Main-----
// --------------



int main(int argc, char *argv[])
{   
    //ros::Time::init();
    ros::init(argc, argv, "point_cloud_map");
    ros::NodeHandle nh;
	
    //ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/royale_camera_driver/point_cloud", 1, callback);
    

    PointCloudMap point_cloud_map;
    point_cloud_map.start();
    return 0;
}


