#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

ros::Publisher vel_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;

const double pi = 3.14159265359;

void move(double speed, double distance, bool isForward);

void rotate(double angular_speed, double angle, bool clockwise);

double degrees_to_radians(double degrees);

void set_orientation(double desired_angle);

void pose_callback(const turtlesim::Pose::ConstPtr &pose_msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "turtle_motion");
	ros::NodeHandle nh;

	vel_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
	pose_subscriber = nh.subscribe("/turtle1/pose", 10, pose_callback);

	ros::Rate loop_rate(0.5);

	set_orientation(degrees_to_radians(120));
	loop_rate.sleep();
	set_orientation(degrees_to_radians(-60));
	loop_rate.sleep();
	set_orientation(0);

	ros::spin();

	return 0;
		
	
}

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;

	if (isForward){
		vel_msg.linear.x = abs(speed);
	}
	else {
		vel_msg.linear.x = -abs(speed);
	}

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	// get initial time
	double t0 = ros::Time::now().toSec();

	// initial distance is zero
	double current_distance = 0;

	ros::Rate loop_rate(100);

	do{
		vel_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while (current_distance < distance);

	// make it stop after going desired distance
	vel_msg.linear.x = 0;
	vel_publisher.publish(vel_msg);
}

void rotate(double angular_speed, double relative_angle, bool clockwise){
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise){
		vel_msg.angular.z = -abs(angular_speed);
	}
	else {
		vel_msg.angular.z = abs(angular_speed);
	}

	// get initial time
	double t0 = ros::Time::now().toSec();

	// initial angle is zero
	double current_angle = 0.0;

	ros::Rate loop_rate(100);

	do{
		vel_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while (current_angle < relative_angle);

	// make it stop after rotating desired angle
	vel_msg.angular.z = 0;
	vel_publisher.publish(vel_msg);
}

double degrees_to_radians(double degrees){
	return degrees * (pi/180.0);
}

void set_orientation(double desired_angle){
	double relative_angle = desired_angle - turtlesim_pose.theta;
	bool clockwise = ((relative_angle < 0)? true:false);
	rotate(abs(relative_angle), abs(relative_angle), clockwise);
}

void pose_callback(const turtlesim::Pose::ConstPtr &pose_msg){
	turtlesim_pose.x = pose_msg->x;
	turtlesim_pose.y = pose_msg->y;
	turtlesim_pose.theta = pose_msg->theta;
}
