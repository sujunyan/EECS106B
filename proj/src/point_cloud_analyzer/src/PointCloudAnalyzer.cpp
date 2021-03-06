#include "PointCloudAnalyzer.hpp"
#include "utils.hpp"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>	
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/search/impl/search.hpp>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/median_filter.h>
//#include <pcl/filters/fast_bilateral.h>
#include <pcl/octree/octree.h>
//#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "std_msgs/String.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sstream>
#include <queue>
#include <math.h>
#include <chrono>


const bool print_flag = true;
//#include <pcl/filters/voxel_grid.h>
pcl::PointXYZRGB findCenter(const PointCloud::ConstPtr& msg);
//float get2Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);
//float get3Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);






//pcl::PointXYZRGB origin_center_point;
//pcl::PointXYZRGB calibrate_point;    //used to find a point, when membrane move to this point,
                                     //adjust it deform
float relative_z = 0.105;             // calibrate point z 
float center_z_depth;

int k = 10;                          // the interval of point cloud to compute the concavity

const int max_column = 224;
const int max_row = 171;
const float radius2d_membrane = 0.065;
const float radius3d_deform_limit = 0.01;

PointCloudAnalyzer::PointCloudAnalyzer(/* args */){
	setup_parameter();
	setup_vis();
}

PointCloudAnalyzer::~PointCloudAnalyzer(){
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
	transformed_memb_ptr = (PointCloud::Ptr) new pcl::PointCloud<pcl::PointXYZRGB>;
	origin_membrane = (PointCloud::Ptr) new pcl::PointCloud<pcl::PointXYZRGB>;
	normals = (pcl::PointCloud<pcl::Normal>::Ptr) new pcl::PointCloud<pcl::Normal>;
	map_ptr = (PointCloud::Ptr) new pcl::PointCloud<pcl::PointXYZRGB>;
	deformed_membrane =  (PointCloud::Ptr)(new pcl::PointCloud<pcl::PointXYZRGB>); 

	std::cout << "Creating point cloud\n\n";

	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	point_cloud_ptr->height = 1;

	viewer = (boost::shared_ptr<pcl::visualization::PCLVisualizer>) (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer1 = (boost::shared_ptr<pcl::visualization::PCLVisualizer>) (new pcl::visualization::PCLVisualizer ("3D Map"));

	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "cloud");
	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "deformed");
	viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, "contact");
    //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (point_cloud_ptr, normals, 10, 0.02, "normals");

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "deformed");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "contact");
	// viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_JET, "cloud");

    std::cout<<"addPlane"<<"\n";
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);

    std::vector<double> origin(3,0);
    origin[2] = 0;
    viewer->addPlane (coeffs, origin[0],origin[1],origin[2], "plane");
	viewer->addCoordinateSystem (0.1);
    

	viewer1->setBackgroundColor (0, 0, 0);
    viewer1->addPointCloud<pcl::PointXYZRGB> (map_ptr, "map");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3 ,"map");
    viewer1->addPlane (coeffs, origin[0], origin[1], origin[2], "baseplane");
    viewer1->addCoordinateSystem(0.1);


	//viewer->initCameraParameters ();
	//viewer->removeCoordinateSystem();
	//std::cout << "Created point cloud\n";
}

/*
void PointCloudAnalyzer::mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}
*/


void PointCloudAnalyzer::armpos_callback(const geometry_msgs::Point::ConstPtr& msg)
{
	static bool first_flag = false; 
	static double first_x,first_y;
	static double offset =  -0.15;
	//printf("x %f\n",msg->x );
	if (first_flag){
		transform_x = msg->x - first_x;
		transform_y = msg->y - first_y;
		transform_z = msg->z - offset;
	}else{
		first_x = msg->x;
		first_y = msg->y;
		first_flag = true;
	}
}


/*
PointCloud::Ptr PointCloudAnalyzer::Genfullmem(const PointCloud::ConstPtr& msg)
{    

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    full_membrane->width = msg->width;
	full_membrane->height = msg->height;
	full_membrane->resize(full_membrane->height*full_membrane->width);
    
    //std::cout << "test track_point"<< track_point.x <<" "<<track_point.y <<" "<<track_point.z<<"\n";
    
    pcl::copyPoint(calibrate_point, center_point);

    calibrate_point.z = relative_z;

    //std::cout<<"center_point"<<" "<<center_point.x<<" "<<center_point.y<<" "<<center_point.z<<"\n";

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
    //std::cout << "generate fullmembrane" <<"\n";
    return full_membrane;
}
*/


PointCloud::Ptr PointCloudAnalyzer::genFullmem(const PointCloud::Ptr& msg)
{    

	
	PointCloud::Ptr full_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);

    full_membrane->width = msg->width;
	full_membrane->height = msg->height;
	full_membrane->resize(msg->height*msg->width);
    
    //std::cout << "test center_point"<< center_point.x <<" "<<center_point.y <<" "<<center_point.z<<"\n";
    
    //pcl::copyPoint(calibrate_point, center_point);

   

    //std::cout<<"center_point"<<" "<<center_point.x<<" "<<center_point.y<<" "<<center_point.z<<"\n";
    //std::cout<<"msg->points.size"<<msg->points.size()<<"\n";
    #if 0
    for (int i=0; i< msg->points.size(); i++)
    {
    	if(pcl::isFinite(msg->points[i])){

         float radius_2d_squared = get2Ddistance(msg->points[i], center_point);
         
         //std::cout<<"radius_2d_squared"<<radius_2d_squared<<"\n";
		 //float radius3d = sqrt(pow(msg->at(c, r).x - track_point.x, 2.0) + pow(msg->at(c, r).y - track_point.y, 2.0) + pow(msg->at(c, r).z - 0.09, 2.0));
         //std::cout<<i<<" "<<msg->points[i].x<<" "<<msg->points[i].y<<" "<<msg->points[i].z<<"\n";
	     
	     
	     //std::cout<<"radius2d_membrane"<<radius2d_membrane<<"\n";
	     if (radius_2d_squared < radius2d_membrane) {

	     	full_membrane->points.push_back(msg->points[i]);
             //full_membrane->points[i].rgb = *reinterpret_cast<float*>(&white);   
         }       
		}			
     }
     #endif

	for (int c =0; c < full_membrane->width; c++){
		for(int r = 0; r <full_membrane->height; r++){
           if(pcl::isFinite(msg->at(c,r))){

           float radius_2d_squared = get2Ddistance(msg->at(c,r), center_point);
           if (radius_2d_squared < radius2d_membrane) {

           	full_membrane->at(c,r).x = msg->at(c,r).x;
           	full_membrane->at(c,r).y = msg->at(c,r).y;
           	full_membrane->at(c,r).z = msg->at(c,r).z; 
           	full_membrane->at(c,r).rgb = *reinterpret_cast<float*>(&white);      
		}
       }
	  }
	}
					
  
   //std::cout << "generate fullmembrane" <<"\n";
    return full_membrane;


}







PointCloud::Ptr PointCloudAnalyzer::genDeformem(const PointCloud::Ptr& msg)
{   
    num_deform = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr deformed_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    deformed_membrane->width = msg->width;
	deformed_membrane->height = msg->height;
	deformed_membrane->resize(full_membrane->height*full_membrane->width);

	for (int c = 0; c < msg->width; c++) {
		for (int r = 0; r < msg->height; r++) {
			if (pcl::isFinite(msg->at(c, r))) {
                    
                    //get 3D deformation area
					float radius3d = get3Ddistance(msg->at(c,r), origin_membrane->at(c,r));

			         if (radius3d > radius3d_deform_limit) {
                    //std::cout<<"deforme"<<"\n";
				        num_deform += 1;
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
	std::cout << "generate normal" <<"\n";
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	
  	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  	ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  	ne.setMaxDepthChangeFactor(0.02f);
  	ne.setNormalSmoothingSize(10.0f);
  	ne.setInputCloud(msg);
  	ne.compute(*normals);
    
    return normals;

}


PointCloud::Ptr PointCloudAnalyzer::genContact(const PointCloud::Ptr& msg)
{ 

 	num_concave= 0;
 	num_plane = 0;

    std::cout<<"start contact"<<endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr contact_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr contact_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    contact_membrane->width = msg->width;
	contact_membrane->height = msg->height;
	contact_membrane->resize(msg->height*msg->width);
     //vector<int> col;
    //vector<int> row;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
     seg.setOptimizeCoefficients (false);
     // Mandatory
     seg.setModelType (pcl::SACMODEL_PLANE);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setDistanceThreshold (0.009f);

     seg.setInputCloud (msg);
     seg.segment (*inliers, *coefficients);

     std::cout<<"inliers->indices.size ()"<<inliers->indices.size ()<<"\n";
     

    if (inliers->indices.size () > 4540 && abs( common_z - min_z) <= 0.025){
     	
		 	for (size_t i = 0; i < inliers->indices.size (); i++){
		 	  	int r = inliers->indices[i] / msg->width;
              	int c = inliers->indices[i] % msg->width;
            	if(abs(msg->at(c,r).z - common_z) <= 0.006){
              		num_plane++;
	              	contact_membrane->at(c, r).x = msg->at(c, r).x;
				  	contact_membrane->at(c, r).y = msg->at(c, r).y;
				  	contact_membrane->at(c, r).z = msg->at(c, r).z;
				  	contact_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&red);
	     	  	}
    	 	}
    	
	}
	else{


	    for (int i=0; i< detectedPoints.size(); i++){
	        int r = detectedPoints[i] / msg->width;
	        int c = detectedPoints[i] % msg->width;

	        if (pcl::isFinite(msg->at(c,r)) && pcl::isFinite(normals->at(c,r))){

				
	        	        //std::cout<<"c_concavity"<<c_concavity<<"r_concavity"<<r_concavity<<endl;
	  					//std::cout<<"abs( common_z - min_z)"<<abs(common_z - min_z)<<endl;
	                	//std::cout<<"abs(msg->at(c,r).z - common_z)"<<abs(msg->at(c,r).z - common_z)<<endl;
	                
						
							//msg->at(c,r).z <= min_z + 0.02){
							// See what local surface area this point covers
							//float surface_area = surfaceElementArea(full_membrane->at(c, r), full_membrane->at(c-1, r), full_membrane->at(c+1, r), full_membrane->at(c, r-1), full_membrane->at(c, r+1));
							//total_concave_surface += surface_area;
							//std::cout<<"enter validation"<<"\n";
						    
							contact_membrane->at(c, r).x = msg->at(c, r).x;
							contact_membrane->at(c, r).y = msg->at(c, r).y;
						    contact_membrane->at(c, r).z = msg->at(c, r).z;
							contact_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&green);
	    				
	    			
	    		}
	     	}
 	}
        
        #if 0 
        if (pcl::isFinite(msg->at(c,r)) && pcl::isFinite(normals->at(c,r))){

			if (pcl::isFinite(msg->at(c-k, r)) && pcl::isFinite(msg->at(c+k, r)) 
				&& pcl::isFinite(normals->at(c-k,r)) && pcl::isFinite(msg->at(c+k, r))) {
				// Now check if vertical neighbours exist. Row-wise.
				if (pcl::isFinite(msg->at(c, r-k)) && pcl::isFinite(msg->at(c, r+k))
					 && pcl::isFinite(normals->at(c, r-k)) && pcl::isFinite(normals->at(c, r+k))) {
    			
    				float c_concavity = concavityOfPair(msg->at(c-k, r), msg->at(c+k, r), normals->at(c-k, r), normals->at(c+k, r));
					float r_concavity = concavityOfPair(msg->at(c, r-k), msg->at(c, r+k), normals->at(c, r-k), normals->at(c, r+k));
                
        	        //std::cout<<"c_concavity"<<c_concavity<<"r_concavity"<<r_concavity<<endl;
  					//std::cout<<"abs( common_z - min_z)"<<abs(common_z - min_z)<<endl;
                	//std::cout<<"abs(msg->at(c,r).z - common_z)"<<abs(msg->at(c,r).z - common_z)<<endl;
                
					if (c_concavity > 0 && r_concavity > 0 && 
						get3Ddistance(msg->at(c,r),origin_ptr->at(c,r)) >= max_dis - 0.025) { // 0.016
						//msg->at(c,r).z <= min_z + 0.02){
						// See what local surface area this point covers
						//float surface_area = surfaceElementArea(full_membrane->at(c, r), full_membrane->at(c-1, r), full_membrane->at(c+1, r), full_membrane->at(c, r-1), full_membrane->at(c, r+1));
						//total_concave_surface += surface_area;
						//std::cout<<"enter validation"<<"\n";
					    num_concave += 1;
						contact_membrane->at(c, r).x = msg->at(c, r).x;
						contact_membrane->at(c, r).y = msg->at(c, r).y;
					    contact_membrane->at(c, r).z = msg->at(c, r).z;
						contact_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&red);
    				}
    			}
    		}
     	}
 	}
    #endif 
    

    //std::cout<<"num_concave"<<num_concave<<"\n";
 	std::cout<<"num_plane"<<num_plane<<"\n";
    
 	std::cout << "generate contact_membrane" <<"\n";
 	return contact_membrane;

}
bool PointCloudAnalyzer::addFlag(){
	static bool add_flag = false;
	static bool once_flag = false;
	const double limit = 0.025;
	const double small_limit = 0.015;

	if(max_dis > limit && once_flag){
		add_flag = true;
		once_flag = false;
	}else{
		add_flag = false;
	}
	if(max_dis < small_limit){
		once_flag = true;
	}
	return add_flag;
}

void PointCloudAnalyzer::addcontact(const PointCloud::Ptr& msg, bool add_flag){
	//add_flag = true;
	if (!add_flag)return;
	int scale = 1;

	const auto center_point = pcl::PointXYZRGB(0,0,0);
	const double max_dis_allowed = 0.05;
	for (int i = 0; i < msg->points.size(); i+=3){
		auto point = msg->points[i];
		if (pcl::isFinite(point)){
			map_ptr->push_back(point);
		}
	}
	#if 0
	for (int c = 0 ; c < msg->width  ; c+=scale) {
		for (int r = 0; r < msg->height ; r+=scale) {
			 if (pcl::isFinite(msg->at(c,r))){
	                  map_ptr->push_back(msg->at(c,r));
	              }
	          }
	      }
	#endif
	//printf("add contact points_num %d\n", cnt);
}




/*
PointCloud::Ptr PointCloudAnalyzer::voxel_grid(const PointCloud::ConstPtr&msg)
{
    PointCloud::Ptr output (new PointCloud);

	
    sor.setInputCloud (msg);
    sor.setLeafSize (voxelgrid_size, voxelgrid_size, voxelgrid_size);
    std::cout<<"voxelgrid_size"<<voxelgrid_size<<"\n";
    sor.filter (*output);

    
    std::cout<<"filter success"<<endl;
    return output;
    //std::cout<<"output size"<<test_smooth->points.size()<<"\n";

}
*/

float PointCloudAnalyzer::getsurfacearea(const PointCloud::Ptr&msg)
{
	//std::cout<<"get surface"<<"\n"<<"\n";
	float tmp_surface_area = 0.0;
	for (int c = 1 ; c < msg->width-1  ; c++) {
		for (int r = 1; r < msg->height-1 ; r++) {
			if (pcl::isFinite(msg->at(c, r)) && pcl::isFinite(msg->at(c, r-1)) 
				&& pcl::isFinite(msg->at(c, r+1))&&pcl::isFinite(msg->at(c+1, r))
				&&pcl::isFinite(msg->at(c-1, r))
				 )
		
		{
			float delta_area = surfaceElementArea(msg->at(c, r), msg->at(c-1, r), msg->at(c+1, r), msg->at(c, r-1), msg->at(c, r+1));
            tmp_surface_area += delta_area;
         }
       }
     }
       return tmp_surface_area;
}

#if 0
void PointCloudAnalyzer::Saveorigin(const PointCloud::Ptr& msg)
{
	//std::cout<<"start origin_membrane";
    	origin_membrane->width = msg->width;
	    origin_membrane->height = msg->height;
	    origin_membrane->resize(msg->height*msg->width);
    	for (int c = 0 ; c < msg->width ; c++) {
			for (int r = 0; r < msg->height; r++) {
				if (pcl::isFinite(msg->at(c,r))){
					origin_membrane->at(c, r).x = msg->at(c, r).x;
			    	origin_membrane->at(c, r).y = msg->at(c, r).y;
					origin_membrane->at(c, r).z = msg->at(c, r).z;
				//std::cout<<" "<<c<<" "<<r<<origin_membrane->at(c, r).x<<origin_membrane->at(c, r).y<<origin_membrane->at(c, r).z<<"\n";
			}
		
        }
      }
        
        std::cout<<"end origin_membrane";
        origin_center_point = findCenter(origin_membrane);
        Isfirst = false;
}
#endif



PointCloud::Ptr PointCloudAnalyzer::transform(const PointCloud::Ptr& msg, float x, float y, float z)
{   
	//std::cout<<"transform start"<<"\n";

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1 (0,3) = x;
	transform_1 (1,3) = y;
	transform_1 (2,3) = z;
	// roate around y by pi
	#if 1
	double sqrt2 = sqrt(1/2);
	transform_1 (0,0) = 0;
	transform_1 (0,1) = -1;
	transform_1 (1,0) = -1;
	transform_1 (1,1) = 0;
	#endif
	transform_1 (2,2) = -1;
	transform_1 = 2.5 * transform_1 ;
	PointCloud::Ptr transform_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*msg,*transform_cloud1,transform_1);
    //std::cout<<"transform succees"<<"\n";
    return transform_cloud1;
}


PointCloud::Ptr PointCloudAnalyzer::medianFilter(const PointCloud::ConstPtr& msg)
{
    PointCloud filter_point_cloud = PointCloud (*msg);
	PointCloud::Ptr filter_ptr = filter_point_cloud.makeShared();
    Median.setInputCloud(filter_ptr);
    Median.setWindowSize(10);
    //std::cout<<"filter size"<<" "<<Median.getWindowSize()<<"\n";

    Median.setMaxAllowedMovement(median_maxallowed_move);
    //std::cout<<"median_maxallowed_move"<<median_maxallowed_move<<endl;
    Median.applyFilter(*filter_ptr);
    //std::cout<<"end median"<<endl;

    for (int i=0; i<filter_ptr->points.size(); i++){
    	//std::cout<<filtered->points[i].x<<filtered->points[i].y<<filtered->points[i].z<<endl;
    	//tmp_point_cloud.points[i].rgb = *reinterpret_cast<float*>(&red);
    	filter_ptr->points[i].rgb = *reinterpret_cast<float*>(&white);
    	//std::cout<<"rgb"<<tmp_ptr->points[i].rgb<<"\n";
    	//std::cout<<
    }

    return filter_ptr;
}


void PointCloudAnalyzer::depthCallback(const PointCloud::ConstPtr& msg){
	/*
		store the orignal PointCloud and comapre it it the incoming 
		data. Return the vector [max_relative_depth, mean_relative_depth]
	*/

	//static PointCloud::Ptr origin_ptr;
	//static bool is_origian_stored = false;

	static int cnt = 0;
	const int wait_times = 1;
	const int filter_size = 3;
	static SimpleFilter max_dis_filter(filter_size);
	static SimpleFilter mean_dis_filter(filter_size);
    
    std::map<double, int> zList;
    
	static vector <double> values (2,0); // create a vector with two values 0
	cnt ++;
	if(cnt < wait_times){ // wait for a sufficient time to get use msg
		printf("In depthCallback: Waiting to get the useful point could, cnt=%d\n",cnt);
	}else if( cnt == wait_times){
		origin_ptr = (new PointCloud(*msg))->makeShared();
		origin_membrane = origin_ptr;
		printf("In depthCallback: set the origin point cloud cnt=%d\n",cnt);
	}else{
		// calculate the relative depth
		double max_dis0 = 0;
		double sum_dis0 = 0;
		double tot_num = 0;
		for (int c = 0; c < msg->width; c++){
			for (int r = 0; r < msg->height; r++){
				pcl::PointXYZRGB pa = msg->at(c,r);
				pcl::PointXYZRGB pb = origin_ptr->at(c,r);

				if(pcl::isFinite(pa) && pcl::isFinite(pb)){
					double dis = get3Ddistance(pa,pb);
					double z = msg->at(c,r).z;
                    
					   if (dis >= 0.009){
					   		zList[std::round(z*1000)/1000.0]++;
					   }
					max_dis0 = max(dis,max_dis0);
					sum_dis0 += dis;
					tot_num ++;
				}
			}
		}

       std::map<double, int>::iterator iter;
       
		int maxvalue = 100;
		double maxkey = 2.0;
		double key = -1;
		double tmp1 = 0;
		for(iter = zList.begin() ; iter!= zList.end(); ++iter)
		{
			//std::cout<<"key"<< iter->first << "value" <<iter->second<<endl;
  			int tmp = 0;
   		    tmp = iter->second;
   		    tmp1 = iter->first;
   			if(tmp >= maxvalue )
  			{
            	maxvalue = tmp;
    			key = iter->first;
   			}

   			if(tmp1 < maxkey && tmp1 != 0)
   			{ 
   				maxkey = tmp1;
   			}
            
		}
        common_z = key;
        min_z = maxkey;

		//max_dis = (1-rate) * max_dis + rate * max_dis0;
		//mean_dis = (1-rate) * mean_dis + rate * sum_dis0/tot_num;
		max_dis = max_dis_filter.filter(max_dis0);
		mean_dis = mean_dis_filter.filter(sum_dis0/tot_num);
		printf("In depthCallback: max_dis=%f mean_dis=%f common_z=%f min_z=%f\n",max_dis,mean_dis,common_z, min_z);
	}

}

void print_used_time( std::chrono::time_point<std::chrono::steady_clock> start){
	auto end = chrono::steady_clock::now();
    cout << "printed by print_used_time used" 
			<< chrono::duration_cast<chrono::milliseconds>(end - start).count()
			<< " ms" << endl;
}

void PointCloudAnalyzer::callback(const PointCloud::ConstPtr& msg)
{
	auto start = chrono::steady_clock::now();
	printf("\n\nStarting callback\n");
	static int cnt = 0;
	const int wait_times = 10;
	if (cnt++ < wait_times){
		printf("In callback, waiting for the useful data %d\n",cnt );
		return;
	}

	print_used_time(start);
    int scale = 2;         // scale is related to complementary factor 

    PointCloud::Ptr down_sampled_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>); 

    pcl::PointCloud<pcl::PointXYZRGB> down_sampled_cloud; 

    down_sampled_cloud.width = msg->width /scale  ;     
    down_sampled_cloud.height = msg->height /scale + 1;
    down_sampled_cloud.resize(down_sampled_cloud.width*down_sampled_cloud.height);

    for( int c = 0; c < msg->width; c+= scale){
    	for( int r = 0; r < msg->height; r+= scale){    
             int a = (int) c/ scale;
             int b = (int) r/ scale;  
            down_sampled_cloud.at(a,b).x = msg->at(c,r).x;
            down_sampled_cloud.at(a,b).y = msg->at(c,r).y;
            down_sampled_cloud.at(a,b).z = msg->at(c,r).z;
            down_sampled_cloud.at(a,b).rgb = *reinterpret_cast<float*>(&white);

      }
    }

    down_sampled_cloud_ptr = down_sampled_cloud.makeShared();     
    
    down_sampled_cloud_ptr = medianFilter(down_sampled_cloud_ptr);

    full_membrane = genFullmem(down_sampled_cloud_ptr);
    print_used_time(start);
    depthCallback(down_sampled_cloud_ptr);
	checkContact();
	if (!is_contact){
		//return;
	}
    stiffnessCallback();

    center_z_depth = findCenter(origin_ptr).z - findCenter(full_membrane).z;
    
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree(0.01f);
    
    octree.setInputCloud(origin_membrane);
    octree.addPointsFromInputCloud();
    octree.switchBuffers();

    //std::cout<<"origin_center_point"<<" "<<origin_center_point.x<<" "<<origin_center_point.y<<" " <<origin_center_point.z<<"\n";
    
    octree.setInputCloud(full_membrane);
    octree.addPointsFromInputCloud();
    
    detectedPoints.clear();
    int numberOfNewPoints = octree.getPointIndicesFromNewVoxels(detectedPoints);
    //std::cout << numberOfNewPoints << " New Points Detected!" << std::endl;
    print_used_time(start);

	deformed_membrane->width = msg->width;
	deformed_membrane->height = msg->height;
	deformed_membrane->resize(full_membrane->height*full_membrane->width);
    for (size_t i = 0; i < detectedPoints.size (); ++i)
    {
    	num_deform = 0;
    	int r = detectedPoints[i] / msg->width;
        int c = detectedPoints[i] % msg->width;
        if(get3Ddistance(full_membrane->at(c,r),origin_membrane->at(c,r)) > radius3d_deform_limit){    
        	 num_deform ++;
    	     full_membrane->points[detectedPoints[i]].rgb = *reinterpret_cast<float*>(&green);
    	     deformed_membrane->points.push_back(full_membrane->points[detectedPoints[i]]);
        }
        else{
        	detectedPoints.erase(detectedPoints.begin() + i);
        }
    }
   bool add_flag = addFlag();
   viewer->updatePointCloud(full_membrane, "cloud");
   publishAll();
   if (!add_flag){ // no need to process the following codes
   		//return;
   }
   // deformed_membrane = genDeformem(full_membrane);
    std::cout<<"num_deform"<<num_deform<<"\n";

    normals = Getnormal(full_membrane);
    
    #if 1
    contact_membrane = genContact(full_membrane);

    //contact_membrane_area = (1-rate)* contact_membrane_area + rate * getsurfacearea(contact_membrane); 

    std::cout<<"contact_membrane_area"<<contact_membrane_area<<endl;

    //full_membrane_area = (1-rate) * full_membrane_area + rate *getsurfacearea(full_membrane);

    //std::cout<<"full_membrane_area"<<full_membrane_area<<'\n';
   #endif 
    //TODO
    //transformed_memb_ptr = transform(contact_membrane, transform_x ,transform_y, transform_z);
    //transformed_memb_ptr = transform(full_membrane, transform_x ,transform_y, transform_z);

    //transformed_memb_ptr = transform(deformed_membrane, transform_x ,transform_y, transform_z);
    auto transformed_memb_ptr1 = transform(contact_membrane, transform_x ,transform_y, transform_z);

    addcontact(transformed_memb_ptr1, add_flag);
    

    
    
    //viewer->updatePointCloud(full_membrane, "cloud");
    //viewer->updatePointCloud(full_membrane, "cloud");
    //viewer->updatePointCloud(deformed_membrane, "deformed");
    viewer->updatePointCloud(contact_membrane, "contact");

    viewer1->updatePointCloud(map_ptr, "map");
	auto end = chrono::steady_clock::now();
    cout << "Callback used" 
			<< chrono::duration_cast<chrono::milliseconds>(end - start).count()
			<< " ms" << endl;
	
   
}

void PointCloudAnalyzer::publishAll(){
	std_msgs::Float32 max_dist;
    std_msgs::Float32 mean_dist;
    std_msgs::Float32 contact_area;
    std_msgs::Float32 full_area;

    //depth.data = center_z_depth;
    max_dist.data = max_dis;
    mean_dist.data = mean_dis;
    contact_area.data = contact_membrane_area;
    full_area.data = full_membrane_area;

    //pub_center_z.publish(depth);
  	pub_contact_area.publish(contact_area);
  	pub_full_area.publish(full_area);
  	pub_maxdis.publish(max_dist);
  	pub_meandis.publish(mean_dist);
}

// function to estimate the stiffness at current time
// use current max_dis and arm position to estimate
void PointCloudAnalyzer::stiffnessCallback(){
	const int filter_size = 5;
	static double last_transform_z = 0;
	static SimpleFilter stiffness_filter(filter_size);
	if(!is_contact){
		stiffness = stiffness_filter.filter(0);
		last_transform_z = transform_z; // record the arm position before contact
		printf("Not contact now Estimated stiffness %f\n", stiffness);
	}else{
		double delta_z = last_transform_z - transform_z;
		double force = max_dis; // TODO force is a function of max_dis, let's assume now it is linear. 
		double stiffness_tmp = force / delta_z;
		stiffness = stiffness_filter.filter(stiffness_tmp);
		printf("Estimated stiffness %f\n", stiffness);
	}
}

// check if the membrane is contact to an object, will set the bool variable is_contact
void PointCloudAnalyzer::checkContact(){
	const double threshold = 0.016;
	if(max_dis > threshold){
		is_contact = true;
	}else{
		is_contact = false;
	}

}

void PointCloudAnalyzer::start(){
    
    transform_x = 0;
    transform_y = 0;
    transform_z = 0;
    auto start = chrono::steady_clock::now();
    auto last_start = start;;
	//ros::Publisher pub_contact = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("contact", 100);
    //pub_center_z = nh.advertise<std_msgs::Float32>("center_z_deform", 1000);

    //pub_center_z = nh.advertise<std_msgs::Float32>("center_z_deform", 1000);   
    pub_contact_area = nh.advertise<std_msgs::Float32>("contact_area", 1000);
    pub_full_area = nh.advertise<std_msgs::Float32>("full_membrane_area", 1000);
    pub_maxdis = nh.advertise<std_msgs::Float32>("max_dis", 1000);
    pub_meandis = nh.advertise<std_msgs::Float32>("mean_dis", 1000);


	point_cloud_sub = nh.subscribe<PointCloud>("/royale_camera_driver/point_cloud", 1, &PointCloudAnalyzer::callback, this);

	arm_sub = nh.subscribe<geometry_msgs::Point>("/hand_pub", 1, &PointCloudAnalyzer::armpos_callback, this);
	//point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/royale_camera_driver/point_cloud", 10, callback );
	ros::Rate r(20); // 100 hz

	while (!viewer->wasStopped () && !viewer1->wasStopped())
	{   
		start = chrono::steady_clock::now();
		#if 0
		cout << "Elapsed time in milliseconds : " 
			<< chrono::duration_cast<chrono::milliseconds>(start - last_start).count()
			<< " ms" << endl;
		last_start = start;
		#endif
		//pcl_conversions::toPCL(ros::Time::now(), contact_membrane->header.stamp);
		viewer->spinOnce (10);
		viewer1->spinOnce (10);
		ros::spinOnce();
		r.sleep(); // TODO might has a bug
	}
}



/********************** Implementation for the SimpleFilter ******************/
// give the data of current time, output the filtered data.
double SimpleFilter::filter(const double & data){
	assert(size >= datas.size());
	if(datas.size() < size){
		datas.push_back(data);
	}else if(datas.size() == size){
		datas.erase(datas.begin());
		datas.push_back(data);
	}
	return averageValue();
}
// return the average value of current datas

double SimpleFilter::averageValue(){
	double value = 0;
	for(auto it = datas.begin(); it!=datas.end(); it++){
		value += *it;
	}
	return value/(double)datas.size();
} 
