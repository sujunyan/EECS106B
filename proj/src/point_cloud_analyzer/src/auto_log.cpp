#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <unistd.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


using namespace std;
// void callback(const std_msgs::Float32::ConstPtr& force_msg, const std_msgs::Float32::ConstPtr& deflection_msg) {
// 	std::cout << "Calling back" <<std::endl;

// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_log_node");
	ros::NodeHandle n;

	ofstream outputFile;
	outputFile.open("/home/liam/catkin_ws/EECS106B/proj/datalog/camera.csv", std::ios_base::app);
	outputFile << "===" << endl;
	outputFile << "force" <<",  "<< "full_membrane_msg" << ", " << "contact_area_msg" << ", " << "mean_dis_msg" <<", " << "max_dis_msg"<< endl;
	// Subscribe to relevant nodes

	// message_filters::Subscriber<std_msgs::Float32> force_sub(nh, "/forcesensor", 1);
	// message_filters::Subscriber<std_msgs::Float32> deflection_sub(nh, "/deflection", 1);


	// TimeSynchronizer<std_msgs::Float32, std_msgs::Float32> sync(force_sub, deflection_sub, 10);
	// sync.registerCallback(boost::bind(&callback, _1, _2));

	// ros::spin();


	while (true) {
		std_msgs::Float32 max_dis_msg;
		//std_msgs::Float32 force_msg;
		//std_msgs::Float32 deflection_msg;
		std_msgs::Float32 contact_area_msg;
		std_msgs::Float32 full_membrane_msg;
		std_msgs::Float32 mean_dis_msg;
		std_msgs::Float32 force;
		//std_msgs::Float32 center_dis_msg;


		// dist_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("/dist", n));
		// for (int i = 0; i < 5; i ++) {
		full_membrane_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("/full_membrane_area", n));
		contact_area_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("/contact_area", n));
		mean_dis_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("/mean_dis", n));
		//center_dis_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("/center_z_deform", n));
		max_dis_msg = *(ros::topic::waitForMessage<std_msgs::Float32>("/max_dis", n));

        force = *(ros::topic::waitForMessage<std_msgs::Float32>("/force", n));


		outputFile << force.data << " ,   "<<full_membrane_msg.data << ",          " << contact_area_msg.data << ",           " << mean_dis_msg.data <<",          " << max_dis_msg.data<< endl;
		// }

		ros::spinOnce();
		// ros::Duration(0.05).sleep();

	}

	return 0;
}