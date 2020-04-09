#include "remoteCmdHandler.h"
#include <iostream>
#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>

using std::cout;
using std::endl;

class WanNode
{
public:
	WanNode();
	~WanNode();
	bool init();
private:
	void cmdFromRemote();
	
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
	mRobotId = nh_private.param<uint16_t>("robot_id",505);
	mServerIp = nh_private.param<std::string>("server_ip","62.234.114.48");
	mServerPort = nh_private.param<uint16_t>("server_port",8617);
	mCmdHandler.setServerAddr(mServerIp, mServerPort);
	mCmdHandler.setRobotId(mRobotId);
	mCmdHandler.bindCallbackFunction(&WanNode::cmdFromRemote,this);
	
	mPubCmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	return mCmdHandler.start();
}

int main(int argc,char**argv) 
{
	ros::init(argc,argv,"remote_wan_node");
	
	WanNode wanNode;
	if(wanNode.init())
		ros::spin();
	return 0;
}
