#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PoseArray.h>
//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define AVERAGE_PRED 1

//Global Variables
double hw = 2.0;
double fl = 4.0;
double bl = 1.0;

std::vector<double> gt_coords (3, 0.0);
std::vector<double> pred_coords (3, 0.0);

int count = 0;
int flag_pred = 0;
int flag_gt = 0;

//Functions

std::vector<geometry_msgs::Point> generate_box_corners(double x, double y, double theta){
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

	pts.push_back(ll);
	pts.push_back(lr);
	pts.push_back(ul);
	pts.push_back(ur);

	return pts;
}

visualization_msgs::Marker getBBMarker(std::string name, int id, float scale, float r){
	
	visualization_msgs::Marker mk;
	mk.header.frame_id = "map";
	mk.header.stamp = ros::Time::now();
	mk.ns = name;
	mk.id = id;
	mk.type = mk.LINE_STRIP;
	mk.action = mk.ADD;
	mk.scale.x = scale;
	mk.color.a = 1.0;
	mk.color.r = r;
	mk.color.g = 1.0;
	mk.color.b = 0.0;

	return mk;
}

std::vector<visualization_msgs::Marker> getBBs(){

	std::vector<visualization_msgs::Marker> v;

	//for ground truth pod position
	if (flag_gt){
		visualization_msgs::Marker mk1 = getBBMarker("PHZ_gt", 1, 0.3, 1.0);
		mk1.points = generate_box_corners(gt_coords[0], gt_coords[1], gt_coords[2]);		
		v.push_back(mk1);
		flag_gt = 0;
	}
	
	//for predicted pod position
	if (flag_pred){
		visualization_msgs::Marker mk2 = getBBMarker("PHZ_pred", 0, 0.2, 0.0);
		mk2.points = generate_box_corners(pred_coords[0], pred_coords[1], pred_coords[2]);
		v.push_back(mk2);
		flag_pred = 0;
	}

	return v;
}

std::vector<double> set_params(const geometry_msgs::PoseStamped::ConstPtr& msg){
	std::vector<double> v (3, 0.0);
	v[0] = msg->pose.position.x;
	v[1] = msg->pose.position.y;

		tf2::Quaternion q(
		msg->pose.orientation.x,
		msg->pose.orientation.y,
		msg->pose.orientation.z,
		msg->pose.orientation.w);
	tf2::Matrix3x3 mat(q);

	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	v[2] = yaw;

	return v;
}

void pod_gt_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	gt_coords = set_params(msg);
	flag_gt = 1;
}

void pod_pred_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	std::vector<double> v = set_params(msg);
	pred_coords[0] = (pred_coords[0]*count + v[0])/(count+1);
	pred_coords[1] = (pred_coords[1]*count + v[1])/(count+1);
	pred_coords[2] = (pred_coords[2]*count + v[2])/(count+1);

	if (AVERAGE_PRED == 1) count++;
	flag_pred = 1;

}


int main(int argc, char **argv){

	ros::init(argc, argv, "marker_visualizer_node");
	ros::NodeHandle n;	

	ros::Subscriber pod_pred_sub = n.subscribe("pod_predicted_laser",1000, pod_pred_CB);
	ros::Subscriber pod_gt_sub = n.subscribe("pod_groundtruth",1000,pod_gt_CB);
	ros::Publisher phz_pub = n.advertise<visualization_msgs::MarkerArray>("phz",10);
	ros::Publisher phz_marker_pub = n.advertise<visualization_msgs::MarkerArray>("estimateLoc", 10);

	visualization_msgs::MarkerArray phz_array;

	ros::Rate loop_rate(10);

	while(ros::ok()) {
		if (flag_pred || flag_gt){		
			phz_array.markers = getBBs();
			phz_pub.publish(phz_array);
		}
		phz_pub.publish(phz_array);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}