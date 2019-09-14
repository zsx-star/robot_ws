#include<can2serial/can2serial.h>
#include<iostream>
#include<ros/ros.h>

using namespace std;

class BaseControl
{
public:
	BaseControl()
	{
	
	}
	~BaseControl()
	{
	
	}
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private)
	{
		mCan = new Can2serial();
		string port_name = nh_private.param<string>("port_name","/dev/ttyUSB0");
		if(!mCan->configPort(port_name))
		{
			ROS_ERROR("[%s] open %s failed!",ros::this_node::getName().c_str(), port_name.c_str());
			return false;
		}
	}
private:
	Can2serial *mCan;
	

};
int main(int argc,char** argv)
{
	ros::init(argc,argv,"base_control_node");
	
	ros::NodeHandle nh, nh_private("~");
	
	BaseControl base_control;
	if(base_control.init(nh,nh_private))
		ros::spin();
	
	return 0;
}

