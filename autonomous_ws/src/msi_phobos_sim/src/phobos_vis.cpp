/*
 * This program is the basic visualisation for the
 * MSI Phobos, the ultra-compact rendition of the
 * 4-wheel differential drive rover "Agastya 2.0".
 * Package under construction.

 * Copyright © 2017 by Dhruv Ilesh Shah
 * dhruv.shah@iitb.ac.in | dhruv.ilesh@gmail.com
 * The IIT-B Mars Rover Team
*/

#ifndef ROS_BASE
	#define ROS_BASE
	#include "ros/ros.h"
	#include "std_msgs/String.h"
#endif

#ifndef TWIST
	#define TWIST
	#include <geometry_msgs/Twist.h>
#endif

#ifndef MARKER
 	#define MARKER
 	#include <visualization_msgs/Marker.h>
#endif

#ifndef DIFF_MSG
 	#define DIFF_MSG
 	#include <msi_phobos_sim/diff_vel.msg>
#endif

#ifndef POSE
 	#define POSE
 	#include <geometry_msgs/Pose.h>
#endif

geometry_msgs::Pose phobos_pos;
geometry_msgs::Twist phobos_vel;
float r = 0.5; // An approximate value of the radius from CoM to wheels


void diff_callback(const msi_phobos_sim::diff_vel &diff_vel_data) {
	// Update the velocity and pose of Phobos given the diff_vel



}

int main(int argc, char **argv) {

	phobos_pos.position.x = 0.0;
	phobos_pos.position.y = 0.0;
	phobos_pos.position.z = 0.0;

	phobos_pos.pose.x = 0.0;
	phobos_pos.pose.y = 0.0;
	phobos_pos.pose.z = 0.0;
	phobos_pos.pose.w = 1.0;


	phobos_vel.linear.x = 0.0;
	phobos_vel.linear.y = 0.0;
	phobos_vel.linear.z = 0.0;
	phobos_vel.angular.x = 0.0;
	phobos_vel.angular.y = 0.0;
	phobos_vel.angular.z = 0.0;





	ros::init(argc, argv, "phobos_frame");
	ros::NodeHandle n;

	ros::Subscriber differential_vel = n.subscribe("cmd_vel", 100, diff_callback);
	ros::Publisher phobos_marker_pub = n.advertise<visualization_msgs::Marker>("phobos_main", 100);

	// Initialising the marker
	uint32_t phobos_shape = visualization_msgs::Marker::CUBE;

	// Running at 10Hz
	update_rate = 10;
	ros::Rate loop_rate(update_rate);
	ros::spinOnce();

	visualization_msgs::Marker phobos_marker;
	phobos_marker.header.frame_id = "/main_frame";
	phobos_marker.ns = "phobos";
	phobos_marker.type = phobos_shape;
	phobos_marker.action = visualization_msgs::Marker::ADD;

	phobos_marker.scale.x = 0.5;
	phobos_marker.scale.y = 0.2;
	phobos_marker.scale.z = 0.1;
	phobos_marker.color.a = 1.0;
	phobos_marker.color.r = 0.0;
	phobos_marker.color.g = 1.0;
	phobos_marker.color.b = 0.0;

	while (ros::ok()) {

		phobos_marker.header.stamp = ros::Time::now();
		// Update the marker position from subscriber
		phobos_marker.pose.position.x = phobos_pos.position.x;
		phobos_marker.pose.position.y = phobos_pos.position.y;
		phobos_marker.pose.position.z = phobos_pos.position.z;

		phobos_marker.pose.orientation.x = phobos_pos.pose.x;
		phobos_marker.pose.orientation.y = phobos_pos.pose.y;
		phobos_marker.pose.orientation.z = phobos_pos.pose.z;
		phobos_marker.pose.orientation.w = phobos_pos.pose.w;

		phobos_marker_pub.publish(phobos_marker);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}