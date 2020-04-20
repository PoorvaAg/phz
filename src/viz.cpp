#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



visualization_msgs::MarkerArray phz_array;
std::vector<visualisation_msgs::Marker> markers(2, );

double hw = 2.0;
double fl = 4.0;
double bl = 1.0;

std::vector<geometry_msgs::Point> generate_points(double x, double y, double theta){
	std::vector<geometry_msgs::Point> pts;
	double R = sqrt(x*x+y*y);

	geometry_msgs::Point ll;
	ll.x = (R-fl)*cos(theta) - hw*sin(theta);
	ll.y = (R-fl)*sin(theta) + hw*cos(theta);
	ll.z = 0.0;
	
	geometry_msgs::Point lr;
	lr.x = (R-fl)*cos(theta) - (-hw)*sin(theta);
	lr.y = (R-fl)*sin(theta) + (-hw)*cos(theta);
	lr.z = 0.0;

	geometry_msgs::Point ul;
	ul.x = (R+bl)*cos(theta) - hw*sin(theta);
	ul.y = (R+bl)*sin(theta) + hw*cos(theta);
	ul.z = 0.0;

	geometry_msgs::Point ur;
	ur.x = (R+bl)*cos(theta) - (-hw)*sin(theta);
	ur.y = (R+bl)*sin(theta) + (-hw)*cos(theta);
	ur.z = 0.0;

	pts.append(ll);
	pts.append(lr);
	pts.append(ul);
	pts.append(ur);

	return pts;
}

visualization_msgs::Marker getBBMarker(string name, int id, float scale, float r){
	
	visualization_msgs::Marker mk;
	mk.header.frame_id = "/map";
	mk.header.stamp = rospy.get_rostime();
	mk.ns = name;
	mk.id = id;
	mk.type = marker.LINE_STRIP;
	mk.action = marker.ADD;
	mk.scale.x = scale;
	mk.color.a = 1.0;
	mk.color.r = r;
	mk.color.g = 1.0;
	mk.color.b = 0.0;

	return mk;
}

void pod_pred_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	double x = msg->pose.position.x;
	double y = msg->pose.position.y;

	tf2::Quaternion q(
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w);
	tf2::Matrix3x3 mat(q);

	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);

	visualization_msgs::Marker mk;
	mk = getBBMarker("PHZ_pred", 0, 0.2, 0.0);
	mk.points = generate_points(x, y, yaw);
}

set_params(const geometry_msgs::PoseStamped::ConstPtr& msg){
	std::
}

void pod_gt_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	gt_x = msg->pose.position.x;
	gt_y = msg->pose.position.y;

	tf2::Quaternion q(
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w);
	tf2::Matrix3x3 mat(q);

	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, );



	
}

visualization_msgs::Marker mk;
	mk = getBBMarker("PHZ_gt", 1, 0.3, 1.0);
	mk.points = generate_points(x, y, yaw);


int main(int argc, char **argv){

	ros::init(argc, argv, "marker_visualizer_node");
	ros::NodeHandle n;	

	ros::Subscriber pod_pred_sub = n.subscribe("pod_predicted_laser",1000, pod_pred_CB);
	ros::Subscriber pod_gt_sub = n.subscribe("pod_groundtruth",1000,pod_gt_CB);

	ros::Rate loop_rate(10);

	while(ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}