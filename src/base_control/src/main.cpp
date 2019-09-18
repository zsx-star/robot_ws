#include<can2serial/can2serial.h>
#include<serial/serial.h>
#include<iostream>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<cmath>

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
		//rotation radius of robot 
		static float R = 0.175; //m
		//The robot rotates around a circle, and the distance of the wheels is L.
		static float L = 2*M_PI*R ;
		//radius of robot wheel 
		static float r = 0.05; //m
		//val = speed*(14.0/2*pi*r)
		static float coff = 14.0/(2*M_PI*r);
		
		//byte2 is mode
		static uint8_t cmdBuf[10] = {0xff,0xfe,0x01};
		
		float linear_x = (cmd->linear.x>1.0||cmd->linear.x<-1.0)?1.0:fabs(cmd->linear.x);
		float linear_y = (cmd->linear.y>1.0||cmd->linear.y<-1.0)?1.0:fabs(cmd->linear.y);
		float angular_z = (cmd->angular.z>180.0||cmd->angular.z<-180.0)?180.0:fabs(cmd->angular.z);
		
		uint16_t x = fabs(linear_x)*coff;
		uint16_t y = fabs(linear_y)*coff;
		uint16_t z = fabs(angular_z)/360.0*L*coff;

		
		cmdBuf[3] = x >> 8;
		cmdBuf[4] = x;
		
		cmdBuf[5] = y >> 8;
		cmdBuf[6] = y;
		
		cmdBuf[7] = z >> 8;
		cmdBuf[8] = z;
		
		cmdBuf[9] = 0x00; //clear
		if(cmd->linear.x < 0)
			cmdBuf[9] |= 0x04;
		if(cmd->linear.y < 0)
			cmdBuf[9] |= 0x02;
		if(cmd->angular.z < 0)
			cmdBuf[9] |= 0x01;
		
//		for(int i=0; i<10; ++i)
//			printf("%x ",cmdBuf[i]);
//		printf("\n");
		
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

