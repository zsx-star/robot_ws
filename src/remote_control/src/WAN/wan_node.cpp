#include "remoteCmdHandler.h"
#include <iostream>
#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <cmath>

using std::cout;
using std::endl;

class WanNode
{
public:
	WanNode();
	~WanNode();
	bool init();
	bool isRunning();
private:
	void cmdFromRemote(controlCmd_t cmd);
	
private:
	ros::Publisher mPubCmd;
	ros::NodeHandle nh,nh_private;
	
	RemoteCmdHandler mCmdHandler;
	int mRobotId;
	std::string mServerIp;
	int mServerPort;
	float mMaxLinearSpeed;
	float mMaxAngularSpeed;
};

WanNode::WanNode():
	nh_private("~")
{
}

WanNode::~WanNode()
{
	mCmdHandler.stop(); 
}

bool WanNode::isRunning()
{
	return ros::ok() && mCmdHandler.isRunning();
}

void WanNode::cmdFromRemote(controlCmd_t cmd)
{
	cout << "xSpeed: " << int(cmd.xSpeed) << "\t zSpeed: " << int(cmd.zSpeed) << endl;
	
	geometry_msgs::Twist twist;
	twist.linear.x = mMaxLinearSpeed*cmd.xSpeed/127;
	twist.angular.z = mMaxAngularSpeed*cmd.zSpeed/127;
	
	mPubCmd.publish(twist);
}

bool WanNode::init()
{
	mRobotId = nh_private.param<int>("robot_id",505);
	mServerIp = nh_private.param<std::string>("server_ip","62.234.114.48");
	mServerPort = nh_private.param<int>("server_port",8617);
	mMaxLinearSpeed = nh_private.param<float>("max_linear_speed",1.0);
	mMaxAngularSpeed = nh_private.param<int>("max_angular_speed",M_PI);
	
	mCmdHandler.setServerAddr(mServerIp, mServerPort);
	mCmdHandler.setRobotId(mRobotId);
	mCmdHandler.registerCallback(&WanNode::cmdFromRemote,this);
	
	mPubCmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	return mCmdHandler.start();
}

int main(int argc,char**argv) 
{
	ros::init(argc,argv,"remote_wan_node");
	
	WanNode wanNode;
	if(!wanNode.init())
		return 0;
	ros::Rate r(20);
	while(wanNode.isRunning())
	{
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
