#include<can2serial/can2serial.h>
#include<serial/serial.h>
#include<iostream>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>

using std::string;
using std::cout;
using std::endl;


class BaseControl
{
public: 
	BaseControl()
	{
	
	}
	~BaseControl()
	{
	
	}
	
	void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
	{
		//x_speed = cmd->linear.x/14*pi*d
		
		//byte2 is mode
		static uint8_t cmdBuf[10] = {0xff,0xfe,0x01};
		uint16_t linear_x = fabs(cmd->linear.x);
		uint16_t linear_y = fabs(cmd->linear.y);
		uint16_t linear_z = fabs(cmd->angular.z);
		
		cmdBuf[3] = linear_x >> 8;
		cmdBuf[4] = linear_x;
		
		cmdBuf[5] = linear_y >> 8;
		cmdBuf[6] = linear_y;
		
		cmdBuf[7] = linear_z >> 8;
		cmdBuf[8] = linear_z;
		
		cmdBuf[9] = 0x00; //clear
		if(cmd->linear.x < 0)
			cmdBuf[9] |= 0x04;
		if(cmd->linear.y < 0)
			cmdBuf[9] |= 0x02;
		if(cmd->angular.z < 0)
			cmdBuf[9] |= 0x01;
		
		mSerial->write(cmdBuf,10);
	}
	
	
	bool init(ros::NodeHandle nh,ros::NodeHandle nh_private)
	{
		mSubCmd = nh.subscribe("/cmd_vel",1,&BaseControl::cmd_callback,this);
		mPubOdom = nh.advertise<nav_msgs::Odometry>("/odom",50);
	
		string port_name = nh_private.param<string>("port_name","/dev/ttyUSB0");
		mUseSerial = nh_private.param<bool>("use_serial",true);
		if(mUseSerial)
			return initSerial(port_name, nh_private.param<int>("baud_rate",115200));
		return initCan(port_name);
	}
	
	bool initCan(string& port_name)
	{
		mCan = new Can2serial();
		
		if(!mCan->configPort(port_name))
		{
			ROS_ERROR("[%s] open %s failed!",ros::this_node::getName().c_str(), port_name.c_str());
			return false;
		}
		return true;
	}
	
	bool initSerial(string& port_name,int baud_rate)
	{
		try 
		{
			mSerial = new serial::Serial(port_name,baud_rate,serial::Timeout::simpleTimeout(10)); 
			if (!mSerial->isOpen())
			{
				std::stringstream output;
				output << "Serial port: " << port_name << " failed to open." << std::endl;
				delete mSerial;
				mSerial = NULL;
				return false;
			} 
			else 
			{
				std::stringstream output;
				output << "Serial port: " << port_name << " opened successfully." << std::endl;
				std::cout << output.str() <<std::endl;
			}
			mSerial->flush();
		} 
		catch (std::exception &e)
		{
			std::stringstream output;
			output << "Error  " << port_name << ": " << e.what();
			std::cout << output.str() <<std::endl;
			return false;
		}
		
		//send 10 bytes random data to enable the robot
		for(uint8_t i=0; i<10; ++i)
			mSerial->write(&i,1);
		
		
		return true;
	}
	
	
	
private:
	ros::Subscriber mSubCmd;
	ros::Publisher mPubOdom;

	Can2serial *mCan;
	serial::Serial *mSerial;
	bool mUseSerial;

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

