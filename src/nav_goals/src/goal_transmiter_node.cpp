#include <ros/ros.h>
#include <iostream>
#include <thread>
#include "fifo.hpp"
#include <geometry_msgs/PoseStamped.h>

#define __NAME__ "goal_transmiter_node"

#pragma pack(push,1)
typedef struct Position
{
    double x,y;
} position_t;

typedef struct Orientation
{
    double x,y,z,w;
} orientation_t;

typedef struct Pose
{
    position_t position;
    orientation_t orientation;
} pose_t;

#pragma pack(pop)

class GoalTransmiter
{
public:
	GoalTransmiter(){}
	bool init()
	{
		ros::NodeHandle nh, nh_private("~");
		std::string fifo_name = nh_private.param<std::string>("fifo_name","");
		if(fifo_name.empty())
		{
			ROS_ERROR("[%s] Please set param fifo_name!",__NAME__);
			return false;
		}

		if(!mFifo.open(fifo_name, "wr"))
			return false;
		
		std::string ns = nh_private.param<std::string>("namespace","");
		std::string topic = ns + "/goal";
		mPubGoal = nh.advertise<geometry_msgs::PoseStamped>(topic,1);
		
		ROS_INFO("[%s] initial ok .", __NAME__);
		return true;
	}
	
	//该线程等待ros退出后向fifo发送空消息，以促使阻塞接收退出
	void exitThread()
	{
		while(ros::ok())
		{
			ros::Duration(0.5).sleep();
		}
		mFifo.send(" ",1);
	}
	
	void run()
	{
		std::thread t(&GoalTransmiter::exitThread,this);
		
		char buf[100];
		while(ros::ok())
		{
			int len = mFifo.receive(buf, 100);
			if(len != sizeof(pose_t))
				continue;
				
			pose_t *pose = (pose_t *)buf;
		
			orientation_t orien = pose->orientation;
			position_t position = pose->position;
		
			std::cout << position.x << "\t" << position.y << "\n";
			std::cout << orien.x << "\t" << orien.y << "\t" << orien.z << "\t" << orien.w << "\n";
			
			geometry_msgs::PoseStamped goal;
			goal.header.stamp = ros::Time::now();
			goal.header.frame_id = "map";
			goal.pose.position.x = position.x;
			goal.pose.position.y = position.y;
			
			goal.pose.orientation.x = orien.x;
			goal.pose.orientation.y = orien.y;
			goal.pose.orientation.z = orien.z;
			goal.pose.orientation.w = orien.w;
			mPubGoal.publish(goal);
		
		}
		ROS_INFO("[%s] exit.", __NAME__);
	}

private:
	Fifo mFifo;
	ros::Publisher mPubGoal;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "goal_transmiter_node");
	
	GoalTransmiter app;
	if(app.init())
	{
		app.run();
	}
	return 0;
}

