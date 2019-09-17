#include<iostream>
#include<ros/ros.h>
#include<std_msgs/Bool.h>

void fun1(const std_msgs::Bool::ConstPtr& msg){}
void fun2(const std_msgs::Bool::ConstPtr& msg){}
void fun3(const std_msgs::Bool::ConstPtr& msg){}
void fun4(const std_msgs::Bool::ConstPtr& msg){}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	std::string in_topic1 = nh_private.param<std::string>("in_topic1","");
	std::string in_topic2 = nh_private.param<std::string>("in_topic2","");
	std::string in_topic3 = nh_private.param<std::string>("in_topic3","");
	std::string in_topic4 = nh_private.param<std::string>("in_topic4","");
	
	std::string out_topic1 = nh_private.param<std::string>("out_topic1","");
	std::string out_topic2 = nh_private.param<std::string>("out_topic2","");
	std::string out_topic3 = nh_private.param<std::string>("out_topic3","");
	std::string out_topic4 = nh_private.param<std::string>("out_topic4","");
	
	ros::Subscriber sub1,sub2,sub3,sub4;
	ros::Publisher pub1,pub2,pub3,pub4;
	
	if(!in_topic1.empty())
		sub1 = nh.subscribe(in_topic1,1,fun1);
	if(!in_topic2.empty())
		sub2 = nh.subscribe(in_topic2,1,fun2);
	if(!in_topic3.empty())
		sub3 = nh.subscribe(in_topic2,1,fun3);
	if(!in_topic4.empty())
		sub4 = nh.subscribe(in_topic2,1,fun4);
		
	if(!out_topic1.empty())
		pub1 = nh.advertise<std_msgs::Bool>(out_topic1,1);
	if(!out_topic2.empty())
		pub2 = nh.advertise<std_msgs::Bool>(out_topic2,1);
	if(!out_topic3.empty())
		pub3 = nh.advertise<std_msgs::Bool>(out_topic3,1);
	if(!out_topic4.empty())
		pub4 = nh.advertise<std_msgs::Bool>(out_topic4,1);
	
	ros::spin();
}
