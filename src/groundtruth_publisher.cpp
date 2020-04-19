#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

#define RAD2DEG 57.295779513

double phz_x;
double phz_y;
double phz_theta ;
geometry_msgs::PoseStamped phz_msg;

double pod_x;
double pod_y;
double pod_theta ;
geometry_msgs::PoseStamped pod_msg;

void set_pod_loc(){
	pod_msg.header.frame_id = "map";
	pod_msg.pose.position.x = pod_x;
	pod_msg.pose.position.y = pod_y;
	pod_msg.pose.position.z = 0;

	tf2::Quaternion quat;
	quat.setRPY( 0, 0, pod_theta);
	pod_msg.pose.orientation= tf2::toMsg(quat);
}

void set_phz_start(){
	double R = sqrt(pod_x*pod_x + pod_y*pod_y) - 4.0;
	phz_msg.header.frame_id = "map";
	phz_msg.pose.position.x = R*cos(pod_theta);
	phz_msg.pose.position.y = R*sin(pod_theta);
	phz_msg.pose.position.z = 0;
	phz_msg.pose.orientation = pod_msg.pose.orientation;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "groundtruth_publisher_node");
	ros::NodeHandle n;
	
	ros::Publisher pod_gt_pub = n.advertise<geometry_msgs::PoseStamped>("pod_groundtruth", 1000);
	ros::Publisher phz_start_gt_pub = n.advertise<geometry_msgs::PoseStamped>("phz_start_groundtruth", 1000);
  
	ros::Rate loop_rate(1);

	n.getParam("/groundtruth/pod1/x_loc", pod_x);
	n.getParam("/groundtruth/pod1/y_loc", pod_y);
	n.getParam("/groundtruth/pod1/theta", pod_theta);

	set_pod_loc();
	set_phz_start();

	while(ros::ok()) {

		pod_msg.header.stamp = ros::Time::now();
		phz_msg.header.stamp = ros::Time::now();
		
		pod_gt_pub.publish(pod_msg);
		phz_start_gt_pub.publish(phz_msg);
		ros::spinOnce();
	  	loop_rate.sleep();
	}

  return 0;

}