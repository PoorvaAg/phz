#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>
#include <std_msgs/String.h>
#include <cmath>

#define RAD2DEG 57.295779513

ros::Publisher detection_pub;
std_msgs::String out_msg;

double target_x = 0.0;
double target_y = 0.0;

double min_dist = 2.0;


void phz_start_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	target_x =  msg->pose.position.x;
	target_y = msg->pose.position.y;
}


void ndt_pose_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	double x = msg->pose.position.x;
	double y = msg->pose.position.y;

	/*
	double z = msg->pose.position.z;

	tf2::Quaternion q(
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w);

	tf2::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	*/

	double dist = sqrt((x-target_x)*(x-target_x)+(y-target_y)*(y-target_y));

	if (dist<=min_dist){
		ROS_INFO("PHZ reached");
		ROS_INFO("POD Location: X: %.2f Y: %.2f", target_x, target_y);
		ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f", msg->header.frame_id.c_str(), x, y);
		out_msg.data = "REACHED";
		detection_pub.publish(out_msg);
	} else {
		ROS_INFO("PHZ not reached");
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "identify_phz_node");
	ros::NodeHandle n;
	
	ros::Subscriber ndt_pose_sub = n.subscribe("ndt_pose", 1000, ndt_pose_CB);
	ros::Subscriber phz_start_sub = n.subscribe("phz_start_groundtruth", 1000, phz_start_CB);
	detection_pub = n.advertise<std_msgs::String>("detected", 1000);

	ros::Rate loop_rate(10);

	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}