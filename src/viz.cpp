#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



visualization_msgs::MarkerArray phz_box_pred;
visualization_msgs::MarkerArray phz_box_gt;

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

void getBBMarker(){
	
	visualization_msgs::Marker mk;
	mk.header.frame_id = "/map"
	mk.header.stamp = rospy.get_rostime()
	mk.ns = "PHZ"
	mk.id = 0
	mk.type = marker.LINE_STRIP
	mk.action = marker.ADD
	mk.scale.x = 0.2
	mk.color.a = 1.0
	mk.color.r = 0.0
	mk.color.g = 1.0
	mk.color.b = 0.0

	mk.points = generate_points()


	/*
	nums = np.random.uniform(-0.03,0.05,1)
	pt1 = Point(x=x_lim[1] + pod_loc_pred[0], y=y_lim[1]+ pod_loc_pred[1])
	pt2 = Point(x=x_lim[0] + pod_loc_pred[0], y=y_lim[1]+ pod_loc_pred[1])
	pt3 = Point(x=x_lim[0] + pod_loc_pred[0], y=y_lim[0]+ pod_loc_pred[1])
	pt4 = Point(x=x_lim[1] + pod_loc_pred[0], y=y_lim[0]+ pod_loc_pred[1])
	marker.points.append(pt1)
	marker.points.append(pt2)
	marker.points.append(pt3)
	marker.points.append(pt4)
	marker.points.append(pt1)
	marker.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, nums))
	phz_array.markers.append(marker)
	*/

	

	marker = Marker()
	marker.header.frame_id = "/map"
	marker.header.stamp = rospy.get_rostime()
	marker.ns = "PHZ_GT"
	marker.id = 1
	marker.type = marker.LINE_STRIP
	marker.action = marker.ADD
	marker.scale.x = 0.3
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	pt1 = Point(x=x_lim[1] + pod_loc_gt[0], y=y_lim[1]+ pod_loc_gt[1])
	pt2 = Point(x=x_lim[0] + pod_loc_gt[0], y=y_lim[1]+ pod_loc_gt[1])
	pt3 = Point(x=x_lim[0] + pod_loc_gt[0], y=y_lim[0]+ pod_loc_gt[1])
	pt4 = Point(x=x_lim[1] + pod_loc_gt[0], y=y_lim[0]+ pod_loc_gt[1])
	marker.points.append(pt1)
	marker.points.append(pt2)
	marker.points.append(pt3)
	marker.points.append(pt4)
	marker.points.append(pt1)

	phz_array.markers.append(marker)

	return phz_array
}





int main(int argc, char **argv){

	ros::init(argc, argv, "marker_visualizer_node");
	ros::NodeHandle n;	

	ros::Subscriber pod_pred_sub = n.subscribe("pod_predicted_laser",1000, pod_pred_CB);
	ros::Subscriber pod_gt_sub = n.subscribe("pod_groundtruth",1000,pod_gt_CB);

	ros::Rate loop_rate(1);

	while(ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}