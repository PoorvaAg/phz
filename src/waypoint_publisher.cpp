#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::Pose wp0;
geometry_msgs::Pose wp1;
geometry_msgs::Pose wp2;
geometry_msgs::Pose wp3;
geometry_msgs::Pose wp4;
geometry_msgs::PoseArray wp;

geometry_msgs::Pose set_pose(double R, double theta){
	geometry_msgs::Pose p;
	tf2::Quaternion quat;
	quat.setRPY( 0, 0, theta);
	p.orientation = tf2::toMsg(quat);
	p.position.x = R*cos(theta);
	p.position.y = R*sin(theta);
	p.position.z = 0;

	return p;

}

void pod_pred_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	double x = msg->pose.position.x;
	double y = msg->pose.position.y;

	tf2::Quaternion q(
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w);
	tf2::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	double R =  sqrt(x*x + y*y);

	wp0 = set_pose(R-3.5, yaw);
	wp1 = set_pose(R-3.0, yaw);
	wp2 = set_pose(R-2.0, yaw);
	wp3 = set_pose(R-1.0, yaw);
	wp4 = msg->pose;

	//aliter: create std::vector<geometry_msgs::Poses> p
	//keep p.push_back(R-x,yaw)

	wp.header.stamp = ros::Time::now();
	wp.header.frame_id = "map";
	wp.poses.push_back(wp0);
	wp.poses.push_back(wp1);
	wp.poses.push_back(wp2);
	wp.poses.push_back(wp3);
	wp.poses.push_back(wp4);

}

int main(int argc, char **argv){

	ros::init(argc, argv, "waypoint_publisher_node");
	ros::NodeHandle n;	

	ros::Subscriber pod_pred_sub = n.subscribe("pod_predicted_laser",1000,pod_pred_CB);
	ros::Publisher wp_pub = n.advertise<geometry_msgs::PoseArray>("waypoints_goal",10);

	ros::Rate loop_rate(1);

	while(ros::ok()) {

		wp_pub.publish(wp);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}