#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher vel_publisher;

void move(double speed, double distance, bool isForward);

int main(int argc, char **argv){
	ros::init(argc, argv, "turtle_motion");
	ros::NodeHandle nh;

	vel_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

	move (2.0, 5.0, 1);

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

	ros::Rate loop_rate(10);

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
