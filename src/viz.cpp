#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>
#include <tuple>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/*
//Global Variables


double hw = 2.0;
double fl = 4.0;
double bl = 1.0;

std::vector<std::tuple<double, double, geometry_msgs::Quaternion>> gt_coords;
std::vector<std::tuple<double, double, geometry_msgs::Quaternion>> pred_coords;
std::tuple<double, double, geometry_msgs::Quaternion> phz_coords;

int flag_pred = 0;
int flag_gt = 0;
int flag_phz = 0;

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


visualization_msgs::Marker getArrowMarker(std::string name, int id, float r, float x, float y, geometry_msgs::Quaternion q){
	
	visualization_msgs::Marker mk;
	mk.header.frame_id = "map";
	mk.header.stamp = ros::Time::now();
	mk.ns = name;
	mk.id = id;
	mk.type = mk.ARROW;
	mk.action = mk.ADD;
	mk.scale.x = 0.5;
	mk.scale.y = 0.05;
	mk.scale.x = 0.1;
	mk.color.a = 1.0;
	mk.color.r = r;
	mk.color.g = 1.0;
	mk.color.b = 0.0;
	mk.pose.orientation = q;
    mk.pose.position.x = x;
    mk.pose.position.y = y;
    mk.pose.position.z = 0;

	return mk;
}

std::vector<visualization_msgs::Marker> getMarkers(){

	std::vector<visualization_msgs::Marker> v;

	int id = 0;
	//for ground truth waypoint markers
	if (flag_gt){

		int n = gt_coords.size();
		if (n>0){
			
			visualization_msgs::Marker mk = getBBMarker("PHZ_gt", id, 0.3, 1.0);
			mk.points = generate_box_corners(std::get<0>(gt_coords[n-1]), std::get<1>(gt_coords[n-1]),std::get<2>(gt_coords[n-1]));
			id++;
			v.push_back(mk);

			for (int i = 0; i<n; i++){
				v.push_back(getArrowMarker("ground truth", id, 1.0, std::get<0>(gt_coords[n-1]), std::get<1>(gt_coords[n-1]),std::get<2>(gt_coords[n-1])));
				id++;
			}
		}

		flag_gt = 0;
	}
	
	//for predicted waypoint markers
	if (flag_pred){
		int n = pred_coords.size();
		if (n>0){
			
			visualization_msgs::Marker mk = getBBMarker("PHZ_pred", id, 0.3, 0.0);
			mk.points = generate_box_corners(std::get<0>(pred_coords[n-1]), std::get<1>(pred_coords[n-1]),std::get<2>(pred_coords[n-1]));
			id++;
			v.push_back(mk);

			for (int i = 0; i<n; i++){
				v.push_back(getArrowMarker("predicted", id, 0.0, std::get<0>(pred_coords[n-1]), std::get<1>(pred_coords[n-1]),std::get<2>(pred_coords[n-1])));
				id++;
			}
		}

		flag_pred = 0;
	}

	//for phz start location
	if (flag_phz){
		int n = pred_coords.size();
		if (n>0){
			
			visualization_msgs::Marker mk = getBBMarker("PHZ_pred", id, 0.3, 0.0);
			mk.points = generate_box_corners(std::get<0>(pred_coords[n-1]), std::get<1>(pred_coords[n-1]),std::get<2>(pred_coords[n-1]));
			id++;
			v.push_back(mk);

			for (int i = 0; i<n; i++){
				v.push_back(getArrowMarker("predicted", id, 0.0, std::get<0>(pred_coords[n-1]), std::get<1>(pred_coords[n-1]),std::get<2>(pred_coords[n-1])));
				id++;
			}
		}

		flag_pred = 0;
	}

	return v;
}

*/

/*
void pod_gt_CB(const geometry_msgs::PoseArray msg){
	std::vector<std::tuple<double, double, geometry_msgs::Quaternion>> coords;
	geometry_msgs::Pose pose;
	for (pose : msg.poses){
		std::tuple<double, double, geometry_msgs::Quaternion> tup(pose.position.x, pose.position.y,pose.orientation);
		coords.push_back(tup);
	}

	gt_coords = coords;
	flag_gt = 1;
}

void pod_pred_CB(const geometry_msgs::PoseArray msg){
	std::vector<std::tuple<double, double, geometry_msgs::Quaternion>> coords;
	geometry_msgs::Pose pose;
	for (pose : msg.poses){
		std::tuple<double, double, geometry_msgs::Quaternion> tup(pose.position.x, pose.position.y,pose.orientation);
		coords.push_back(tup);
	}

	pred_coords = coords;
	flag_pred = 1;

}

void phz_gt_CB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	std::tuple<double, double, geometry_msgs::Quaternion> tup(pose.position.x, pose.position.y,pose.orientation);
	phz_coords = tup;
	flag_phz = 1;

}
*/


int main(int argc, char **argv){

	ros::init(argc, argv, "marker_visualizer_node");
	ros::NodeHandle n;	

	// ros::Subscriber pod_pred_sub = n.subscribe("waypoints_goal",1000, pod_pred_CB);
	// ros::Subscriber pod_gt_sub = n.subscribe("gt_waypoints_goal",1000,pod_gt_CB);
	// ros::Subscriber phz_start_sub = n.subscribe("phz_start_groundtruth",1000,phz_gt_CB);

	ros::Publisher phz_pub = n.advertise<visualization_msgs::MarkerArray>("phz",10);
	ros::Publisher phz_marker_pub = n.advertise<visualization_msgs::MarkerArray>("estimateLoc", 10);

	visualization_msgs::MarkerArray phz_array;

	ros::Rate loop_rate(10);

	while(ros::ok()) {

		/*
		if (flag_pred || flag_gt || flag_phz ){		
			phz_array.markers = getMarkers();
			phz_pub.publish(phz_array);
		}
		phz_pub.publish(phz_array);
		*/
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}