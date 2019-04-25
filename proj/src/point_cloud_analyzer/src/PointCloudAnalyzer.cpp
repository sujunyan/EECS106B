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
#include <pcl/features/integral_image_normal.h>

//#include <pcl/filters/voxel_grid.h>
pcl::PointXYZRGB findCenter(const PointCloud::ConstPtr& msg);
float get2Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);
float get3Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);
float radius2d_membrane = 0.0012;
float radius3d_deform_limit = 0.028;

PointCloud::Ptr full_membrane;
PointCloud::Ptr deformed_membrane;
pcl::PointCloud<pcl::Normal>::Ptr normals;
pcl::PointXYZRGB center_point;

pcl::PointXYZRGB calibrate_point;    //these two used to find a point, when membrane move to this point,
                                     //adjust it deform
float relative_z = 0.09;              





PointCloudAnalyzer::PointCloudAnalyzer(/* args */)
{
	setup_parameter();
	setup_vis();
}

PointCloudAnalyzer::~PointCloudAnalyzer()
{
}

void PointCloudAnalyzer::setup_parameter(){
	std::cout<<"setting up parameters \n";
    use_case_pub = nh.advertise<std_msgs::String>("/use_case", 1);
    max_filter_pub = nh.advertise<std_msgs::Float32>("/max_filter", 1);
    min_filter_pub = nh.advertise<std_msgs::Float32>("/min_filter", 1);
    exposure_pub = nh.advertise<std_msgs::UInt32>("/expo_time", 1);
	sleep(5);

	std_msgs::String mode_msg;
	mode_msg.data = "MODE_5_45FPS_500";
	use_case_pub.publish(mode_msg);

    std_msgs::Float32 max_filter_msg;
	max_filter_msg.data = 0.3;
	max_filter_pub.publish(max_filter_msg);

	std_msgs::Float32 min_filter_msg;
	min_filter_msg.data = 0.06;
	min_filter_pub.publish(min_filter_msg);

	std_msgs::UInt32 expo_time_msg;
	expo_time_msg.data = 50;
	exposure_pub.publish(expo_time_msg);
	sleep(3);

	ros::spinOnce();	
}


void PointCloudAnalyzer::setup_vis(){

	std::cout<<"setting up vis \n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	std::cout << "Creating point cloud\n\n";

	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	point_cloud_ptr->height = 1;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer = viewer0;
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "deformed");
    //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr, normals, 10, 0.02, "normals");

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "deformed");
    
	// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_JET, "cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->removeCoordinateSystem();

	std::cout << "Created point cloud\n";
}


PointCloud::Ptr PointCloudAnalyzer::Genfullmem(const PointCloud::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    full_membrane->width = msg->width;
	full_membrane->height = msg->height;
	full_membrane->resize(full_membrane->height*full_membrane->width);
    
    //std::cout << "test track_point"<< track_point.x <<" "<<track_point.y <<" "<<track_point.z<<"\n";
    
    center_point = findCenter(full_membrane);

    pcl::copyPoint(calibrate_point, center_point);

    calibrate_point.z = relative_z;

    for (int c = 0; c < msg->width; c++) {
		for (int r = 0; r < msg->height; r++) {
			if (pcl::isFinite(full_membrane->at(c, r))) {
				
				float radius_2d_squared = get2Ddistance(msg->at(c,r), center_point);
				//float radius3d = sqrt(pow(msg->at(c, r).x - track_point.x, 2.0) + pow(msg->at(c, r).y - track_point.y, 2.0) + pow(msg->at(c, r).z - 0.09, 2.0));
                
				if (radius_2d_squared < radius2d_membrane) {

					full_membrane->at(c, r).x = msg->at(c, r).x;
					full_membrane->at(c, r).y = msg->at(c, r).y;
					full_membrane->at(c, r).z = msg->at(c, r).z;	
					full_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&white);

					
			}
		}
	}  
	
  }
    return full_membrane;
}




PointCloud::Ptr PointCloudAnalyzer::Gendeformem(const PointCloud::Ptr& msg)
{   

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr deformed_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    deformed_membrane->width = msg->width;
	deformed_membrane->height = msg->height;
	deformed_membrane->resize(full_membrane->height*full_membrane->width);

	for (int c = 0; c < msg->width; c++) {
		for (int r = 0; r < msg->height; r++) {
			if (pcl::isFinite(msg->at(c, r))) {
                    
                    //get 3D deformation area
					float radius3d = get3Ddistance(msg->at(c,r), calibrate_point);

			         if (radius3d < radius3d_deform_limit) {
                     
                    deformed_membrane->at(c, r).x = msg->at(c, r).x;
					deformed_membrane->at(c, r).y = msg->at(c, r).y;
					deformed_membrane->at(c, r).z = msg->at(c, r).z;

	                deformed_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&green);

	                 }
			
	             }
           }
     }
    return deformed_membrane;
}

pcl::PointCloud<pcl::Normal>::Ptr PointCloudAnalyzer::Getnormal(PointCloud::Ptr& msg)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  	ne.setMaxDepthChangeFactor(0.02f);
  	ne.setNormalSmoothingSize(10.0f);
  	ne.setInputCloud(msg);
  	ne.compute(*normals);
    
    return normals;

}



void PointCloudAnalyzer::callback(const PointCloud::ConstPtr& msg)
{
	std::cout << "PointCloud message recieved\n";

    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
   
        
    full_membrane = Genfullmem(msg);
    normals = Getnormal(full_membrane);
    deformed_membrane = Gendeformem(full_membrane);
   


   
    viewer->updatePointCloud(full_membrane, "cloud");
    viewer->updatePointCloud(deformed_membrane, "deformed");

    viewer->removePointCloud("normals", 0);
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (full_membrane, normals, 10, 0.005, "normals");
}

void PointCloudAnalyzer::start(){
	point_cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/royale_camera_driver/point_cloud", 1, &PointCloudAnalyzer::callback, this);
	//point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/royale_camera_driver/point_cloud", 10, callback );
	ros::Rate r(10); // 10 hz
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		ros::spinOnce();
		r.sleep(); // TODO might has a bug
	}
}



