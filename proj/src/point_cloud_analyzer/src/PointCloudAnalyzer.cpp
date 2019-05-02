#include "PointCloudAnalyzer.hpp"

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

//#include <pcl/filters/radius_outlier_removal.h>
#include <fstream>

#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "std_msgs/String.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sstream>
#include <queue>


//#include <pcl/filters/voxel_grid.h>
pcl::PointXYZRGB findCenter(const PointCloud::ConstPtr& msg);
//float get2Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);
//float get3Ddistance(const pcl::PointXYZRGB& a, const pcl::PointXYZRGB& b);






//pcl::PointXYZRGB origin_center_point;
//pcl::PointXYZRGB calibrate_point;    //used to find a point, when membrane move to this point,
                                     //adjust it deform
float relative_z = 0.105;             // calibrate point z 
float center_z_depth;
bool Isfirst;

int k = 10;                          // the interval of point cloud to compute the concavity





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

	Isfirst = true;
	//Iscontact = False;
}


void PointCloudAnalyzer::setup_vis(){

	std::cout<<"setting up vis \n";
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	map_ptr = (PointCloud::Ptr) new pcl::PointCloud<pcl::PointXYZRGB>;
	transformed_memb_ptr = (PointCloud::Ptr) new pcl::PointCloud<pcl::PointXYZRGB>;
	origin_membrane = (PointCloud::Ptr) new pcl::PointCloud<pcl::PointXYZRGB>;
	test_smooth = (PointCloud::Ptr)new pcl::PointCloud<pcl::PointXYZRGB>;
    test_stat = (PointCloud::Ptr)new pcl::PointCloud<pcl::PointXYZRGB>;
    //full_membrane = (PointCloud::Ptr)new pcl::PointCloud<pcl::PointXYZRGB>;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	std::cout << "Creating point cloud\n\n";

	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
	point_cloud_ptr->height = 1;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Map"));

	viewer = viewer0;
	viewer1 = viewer2;
	viewer->setBackgroundColor (255, 255, 255);
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
    viewer->addPlane (coeffs, origin[0],origin[1],origin[2], "plane");
	viewer->addCoordinateSystem (0.1);
    


	viewer1->setBackgroundColor (0, 0, 0);
    viewer1->addPointCloud<pcl::PointXYZRGB> (map_ptr, "map");
    viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"map");

    
    viewer1->addPlane (coeffs, origin[0], origin[1], origin[3], "baseplane");
    viewer1->addCoordinateSystem(0.1);


	//viewer->initCameraParameters ();
	//viewer->removeCoordinateSystem();
	//std::cout << "Created point cloud\n";
}

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



void PointCloudAnalyzer::armpos_callback(const geometry_msgs::Point::ConstPtr& msg)
{
	transform_x = msg->x;
	transform_y = msg->y;
	transform_z = msg->z;
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


PointCloud::Ptr PointCloudAnalyzer::Genfullmem(const PointCloud::Ptr& msg)
{    

	
	PointCloud::Ptr full_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);

    //full_membrane->width = msg->width;
	//full_membrane->height = msg->height;
	//full_membrane->resize(full_membrane->height*full_membrane->width);
    
    //std::cout << "test center_point"<< center_point.x <<" "<<center_point.y <<" "<<center_point.z<<"\n";
    
    //pcl::copyPoint(calibrate_point, center_point);

   

    //std::cout<<"center_point"<<" "<<center_point.x<<" "<<center_point.y<<" "<<center_point.z<<"\n";
    //std::cout<<"msg->points.size"<<msg->points.size()<<"\n";

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
             full_membrane->points[i].rgb = *reinterpret_cast<float*>(&white);   
         }       
		}			
     }
					
  
    std::cout << "generate fullmembrane" <<"\n";
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
                    //std::cout<<"deforme"<<"\n";
                    deformed_membrane->at(c, r).x = msg->at(c, r).x;
					deformed_membrane->at(c, r).y = msg->at(c, r).y;
					deformed_membrane->at(c, r).z = msg->at(c, r).z;

	                deformed_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&green);
	                 }
			
	             }
           }
     }
    //std::cout << "generate deformation" <<"\n";
    
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


PointCloud::Ptr PointCloudAnalyzer::Gencontact(const PointCloud::Ptr& msg)
{ 
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr contact_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
    contact_membrane->width = msg->width;
	contact_membrane->height = msg->height;
	contact_membrane->resize(msg->height*msg->width);

    for (int c = k ; c < 224 - k ; c++) {
		for (int r = k; r < 171 -k ; r++) {
			// Let's do column wise first (horizontal neighbours). Check if horizontal neighbours exist
			if (pcl::isFinite(msg->at(c,r)) && pcl::isFinite(normals->at(c,r))){

			if (pcl::isFinite(msg->at(c-k, r)) && pcl::isFinite(msg->at(c+k, r)) 
				&& pcl::isFinite(normals->at(c-k,r)) && pcl::isFinite(msg->at(c+k, r))) {
				// Now check if vertical neighbours exist. Row-wise.
				if (pcl::isFinite(msg->at(c, r-k)) && pcl::isFinite(msg->at(c, r+k))
					 && pcl::isFinite(normals->at(c, r-k)) && pcl::isFinite(normals->at(c, r+k))) 
				{

					//std::cout<<"test xyz"<<msg->at(c-k,r).x<<" "<<msg->at(c-k,r).y<<" "<<msg->at(c-k,r).z<<"\n";
					//std::cout<<"test normal"<<normals->at(c-k,r).normal_x<<" "<<normals->at(c-k,r).normal_y<<" "<<normals->at(c-k,r).normal_z<<"\n";
					float c_concavity = concavityOfPair(msg->at(c-k, r), msg->at(c+k, r), normals->at(c-k, r), normals->at(c+k, r));

					float r_concavity = concavityOfPair(msg->at(c, r-k), msg->at(c, r+k), normals->at(c, r-k), normals->at(c, r+k));

					//std::cout<<"c_concavity"<<' '<<c_concavity<<' '<<"r_concavity"<<r_concavity<<"\n";
                    

                    if (c_concavity > 0 && r_concavity > 0) {
						// See what local surface area this point covers
						//float surface_area = surfaceElementArea(full_membrane->at(c, r), full_membrane->at(c-1, r), full_membrane->at(c+1, r), full_membrane->at(c, r-1), full_membrane->at(c, r+1));
						//total_concave_surface += surface_area;
						//std::cout<<"enter validation"<<"\n";
						contact_membrane->at(c, r).x = msg->at(c, r).x;
					    contact_membrane->at(c, r).y = msg->at(c, r).y;
					    contact_membrane->at(c, r).z = msg->at(c, r).z;
						contact_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&red);
					}
				}
			}
		 } 
 		}
 	}
 	//std::cout << "generate contact_membrane" <<"\n";
 	return contact_membrane;
}



void PointCloudAnalyzer::addcontact(const PointCloud::Ptr& msg)
{
	for (int c = 0 ; c < 224  ; c++) {
		for (int r = 0; r < 171  ; r++) {
			 if (pcl::isFinite(msg->at(c,r))){
			 	     //std::cout<<"if enter add"<<"\n";
	                  map_ptr->push_back(msg->at(c,r));
	                 //std::cout<<"if add success"<<"\n";
	              }
	          }
	      }
}



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

float PointCloudAnalyzer::getsurfacearea(const PointCloud::Ptr&msg)
{
	//std::cout<<"get surface"<<"\n"<<"\n";
	float tmp_surface_area = 0.0;
	for (int c = 1 ; c < 224-1  ; c++) {
		for (int r = 1; r < 171-1 ; r++) {
			if (pcl::isFinite(full_membrane->at(c, r)) && pcl::isFinite(full_membrane->at(c, r-1)) 
				&& pcl::isFinite(full_membrane->at(c, r+1))&&pcl::isFinite(full_membrane->at(c+1, r))
				&&pcl::isFinite(full_membrane->at(c-1, r))
				 )
		
		{
			float delta_area = surfaceElementArea(full_membrane->at(c, r), full_membrane->at(c-1, r), full_membrane->at(c+1, r), full_membrane->at(c, r-1), full_membrane->at(c, r+1));
            tmp_surface_area += delta_area;
         }
       }
     }
       return tmp_surface_area;
}

void PointCloudAnalyzer::Saveorigin(const PointCloud::Ptr& msg)
{
	//std::cout<<"start origin_membrane";
    	origin_membrane->width = msg->width;
	    origin_membrane->height = msg->height;
	    origin_membrane->resize(msg->height*msg->width);
    	for (int c = 0 ; c < 224 ; c++) {
		for (int r = 0; r < 171; r++) {
			if (pcl::isFinite(msg->at(c,r))){

				origin_membrane->at(c, r).x = msg->at(c, r).x;
			    origin_membrane->at(c, r).y = msg->at(c, r).y;
				origin_membrane->at(c, r).z = msg->at(c, r).z;
				//std::cout<<" "<<c<<" "<<r<<origin_membrane->at(c, r).x<<origin_membrane->at(c, r).y<<origin_membrane->at(c, r).z<<"\n";
			}
		
        }
      }
      
        //std::cout<<"end origin_membrane";
        origin_center_point = findCenter(origin_membrane);
        Isfirst = false;
}

PointCloud::Ptr PointCloudAnalyzer::transform(const PointCloud::Ptr& msg, float x, float y, float z)
{   
	//std::cout<<"transform start"<<"\n";

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1 (0,3) = x;
	transform_1 (1,3) = y;
	transform_1 (2,3) = z;
	PointCloud::Ptr transform_cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*msg,*transform_cloud1,transform_1);
    //std::cout<<"transform succees"<<"\n";
    return transform_cloud1;
}




void PointCloudAnalyzer::callback(const PointCloud::ConstPtr& msg)
{
	//std::cout << "PointCloud message recieved\n";

    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    

    
    //pcl::copyPointCloud(*msg, *full_membrane);
    #if 0
    full_membrane->width = msg->width;
	full_membrane->height = msg->height;
	full_membrane->resize(msg->height*msg->width);

    for (int c = 0 ; c < 224  ; c++) {
		for (int r = 0; r < 171  ; r++) {
			if (pcl::isFinite(msg->at(c,r))){
                 
			    full_membrane->at(c, r).x = msg->at(c, r).x;
			    full_membrane->at(c, r).y = msg->at(c, r).y;
				full_membrane->at(c, r).z = msg->at(c, r).z;
				full_membrane->at(c,r).rgb = *reinterpret_cast<float*>(&white);
			}
		}
	}
	#endif
    
    /*
    filtered = voxel_grid(msg);
    for (int i=0; i<filtered->points.size(); i++){
    	//std::cout<<filtered->points[i].x<<filtered->points[i].y<<filtered->points[i].z<<endl;
    	filtered->points[i].rgb = *reinterpret_cast<float*>(&white);
    }
    */
	PointCloud tmp_point_cloud = PointCloud (*msg);
	PointCloud::Ptr tmp_ptr = tmp_point_cloud.makeShared();
    Median.setInputCloud(tmp_ptr);
    Median.setMaxAllowedMovement(0.01);
    std::cout<<"start median"<<endl;
    Median.applyFilter(*tmp_ptr);
    std::cout<<"end median"<<endl;

    for (int i=0; i<tmp_point_cloud.points.size(); i++){
    	//std::cout<<filtered->points[i].x<<filtered->points[i].y<<filtered->points[i].z<<endl;
    	tmp_point_cloud.points[i].rgb = *reinterpret_cast<float*>(&red);
    }

   // full_membrane = Genfullmem(filtered);
    
   
 
    /*
    if(Isfirst && findCenter(full_membrane).z!=0)
    {
    std::cout<<"start save"<<"\n";
    //std::cout<<"findCenter(msg).z"<<findCenter(full_membrane).z<<"\n";
    Saveorigin(full_membrane);
    //origin_membrane_area = getsurfacearea(origin_membrane);
    //std::cout<<"origin_membrane_area"<<origin_membrane_area<<'\n';
    }
    */
    
    /*
    std::cout<<"origin_center_point"<<" "<<origin_center_point.x<<" "<<origin_center_point.y<<" " <<origin_center_point.z<<"\n";
    

    full_membrane_area = getsurfacearea(full_membrane);
    std::cout<<"full_membrane_area"<<full_membrane_area<<'\n';


    center_z_depth = origin_center_point.z - findCenter(full_membrane).z;
    

    deformed_membrane = Gendeformem(full_membrane);
    
    normals = Getnormal(full_membrane);
    contact_membrane = Gencontact(deformed_membrane);
    transformed_memb_ptr = transform(contact_membrane, transform_x ,transform_y, transform_z);

    addcontact(transformed_memb_ptr);
    */

    viewer->updatePointCloud(tmp_ptr, "cloud");
    std::cout<<"update successful"<<endl;
    //viewer->updatePointCloud(full_membrane, "cloud");
    //viewer->updatePointCloud(full_membrane, "cloud");
    //viewer->updatePointCloud(deformed_membrane, "deformed");
    //viewer->updatePointCloud(contact_membrane, "contact");
    //viewer1->updatePointCloud(map_ptr, "map");
    //viewer->removePointCloud("normals", 0);
	//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (full_membrane, normals, 500, 0.02, "normals");
	//viewer->removePointCloud("normals", 0);
    //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (full_membrane, normals, 10, 0.005, "normals");



    //std_msgs::Float32 depth;
    //depth.data = center_z_depth;
    //pub_center_z.publish(depth);

    //ros::spinOnce();
    

    
}

void PointCloudAnalyzer::start(){
    
    transform_x = 0;
    transform_y = 0;
    transform_z = 0;
	//ros::Publisher pub_contact = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("contact", 100);
    pub_center_z = nh.advertise<std_msgs::Float32>("center_z_deform", 1000);

	point_cloud_sub = nh.subscribe<PointCloud>("/royale_camera_driver/point_cloud", 1, &PointCloudAnalyzer::callback, this);

	arm_sub = nh.subscribe<geometry_msgs::Point>("/hand_pub", 1, &PointCloudAnalyzer::armpos_callback, this);
	//point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/royale_camera_driver/point_cloud", 10, callback );
	ros::Rate r(10); // 10 hz

	while (!viewer->wasStopped () && !viewer1->wasStopped())
	{   
		//pcl_conversions::toPCL(ros::Time::now(), contact_membrane->header.stamp);
		viewer->spinOnce (100);
		viewer1->spinOnce (100);
		ros::spinOnce();
		r.sleep(); // TODO might has a bug
		//pub.publish (*contact_membrane);
		//ros::spinOnce();
		//loop_rate.sleep ();
	}
}



