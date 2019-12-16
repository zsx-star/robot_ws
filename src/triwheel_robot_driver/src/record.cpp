#include<iostream>
#include<ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iomanip>
using namespace std;

ofstream of_odom;

void odomCallback(const nav_msgs::Odometry::ConstPtr& cmd)
{
	of_odom<< setiosflags(ios::fixed) << setprecision(6);
	auto pose = cmd->pose.pose;
	auto speed = cmd->twist.twist;
	tf::Quaternion q;
	tf::quaternionMsgToTF( cmd->pose.pose.orientation, q);
	double roll,yaw,pitch;
	tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
	
	
	of_odom << pose.position.x<< "\t" <<  pose.position.y << "\t" <<  pose.position.z << "\t"
			<< roll << "\t" << pitch << "\t" << yaw << "\t"
			<< speed.linear.x << "\t" << speed.linear.y << "\t" << speed.linear.z << "\t" 
			<< speed.angular.x << "\t" << speed.angular.y << "\t" << speed.angular.z << "\n" ;
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "record_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/wheel_odom",10, &odomCallback);
	if(argc < 2)
	{
		ROS_ERROR("please input file name");
		return 0;
	}
	of_odom.open(argv[1]);
	ros::Duration(2.0).sleep();
	ROS_INFO("start record. ");
	ros::spin();
	of_odom.close();
	
	return 0;
}
