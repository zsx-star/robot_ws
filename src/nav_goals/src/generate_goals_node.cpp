#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <iostream>
#include <tinyxml2.h>
#include<boost/filesystem.hpp>
using namespace tinyxml2;
namespace fs = boost::filesystem;

#define __NAME__ "generate_goals_node"

class GenerateGoals
{
public:
	GenerateGoals()
	{
	
	}
	~GenerateGoals()
	{
	
	}
	bool init()
	{
		ros::NodeHandle nh, nh_private("~");
		mGoalsFileName = nh_private.param<std::string>("goals_file_name","");
		if(mGoalsFileName.empty())
		{
			ROS_ERROR("[%s] Please set goals_file_name in launch file!", __NAME__);
			return false;
		}
		//是否为追加编辑模式,　否则为写模式
		mIsAddMode = nh_private.param<bool>("is_add_mode",true);
		if(mIsAddMode)
		{
			//追加模式, 先载入原有文件
			XMLError res = mXmlDoc.LoadFile(mGoalsFileName.c_str());
			if(XML_ERROR_FILE_NOT_FOUND == res)
			{
				ROS_ERROR("[%s] %s not found!", __NAME__, mGoalsFileName.c_str());
				return false;
			}
			else if(XML_SUCCESS != res)
			{
				ROS_ERROR("[%s] Parse %s failed!", __NAME__, mGoalsFileName.c_str());
				return false;
			}
		}
		else
		{
			std::string file_dir = mGoalsFileName.substr(0,mGoalsFileName.find_last_of("/"));
			if(!fs::exists(fs::path(file_dir)))
			{
				ROS_ERROR("[%s] %s is not exist!", __NAME__, file_dir.c_str());
				return false;
			}
			
			//添加xml声明
			tinyxml2::XMLDeclaration* declaration = mXmlDoc.NewDeclaration();
			mXmlDoc.InsertFirstChild(declaration); //在最前插入声明
		}
		
		sub_goal_ = nh.subscribe("/move_base_simple/goal" ,1,&GenerateGoals::goal_callback,this);
		return true;
	}
	
	void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& goal)
	{
		if(goal->header.frame_id != "map")
		{
			ROS_ERROR("[%s] The goal point not in the map frame, but %s!",__NAME__, goal->header.frame_id.c_str());
			return;
		}
		static int seq = 0;
		if(!mIsAddMode)
		{
			//创建根节点
			tinyxml2::XMLElement* goalsNode = mXmlDoc.NewElement("Goals");
			mXmlDoc.InsertEndChild(goalsNode);  //在最后插入根节点
			mIsAddMode = true;
		}
		
		tinyxml2::XMLElement *pRoot=mXmlDoc.RootElement(); //获取根节点
		
		//插入子节点
		tinyxml2::XMLElement* goalNode = mXmlDoc.NewElement("Goal");
		pRoot->InsertEndChild(goalNode);
		
		//添加属性
		goalNode->SetAttribute("id", std::to_string(seq++).c_str());
		goalNode->SetAttribute("name","unknown");
		
		//为goal添加position
		tinyxml2::XMLElement* positionNode = mXmlDoc.NewElement("Position");
		goalNode->InsertFirstChild(positionNode);
		const geometry_msgs::Point& point = goal->pose.position;
		positionNode->SetAttribute("x", std::to_string(point.x).c_str());
		positionNode->SetAttribute("y", std::to_string(point.y).c_str());
		
		//为goal添加orientation
		tinyxml2::XMLElement* orientationNode = mXmlDoc.NewElement("Orientation");
		goalNode->InsertFirstChild(orientationNode);
		const geometry_msgs::Quaternion& orientation = goal->pose.orientation;
		orientationNode->SetAttribute("x", std::to_string(orientation.x).c_str());
		orientationNode->SetAttribute("y", std::to_string(orientation.y).c_str());
		orientationNode->SetAttribute("z", std::to_string(orientation.z).c_str());
		orientationNode->SetAttribute("w", std::to_string(orientation.w).c_str());
		
		ROS_INFO("[%s] add goal x:%.3f\ty:%.3f", __NAME__, point.x, point.y);
	}
	
	bool save()
	{
		bool ok = mXmlDoc.SaveFile(mGoalsFileName.c_str());
		ROS_INFO("[%s] Save goals in %s %s", __NAME__, mGoalsFileName.c_str(),ok?"ok.":"failed!");
		return ok;
	}
	
private:
	ros::Subscriber sub_goal_;

	tinyxml2::XMLDocument mXmlDoc;
	std::string mGoalsFileName;
	bool mIsAddMode;
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "generate_goals_node");
	GenerateGoals app;
	if(app.init())
	{
		ros::spin();
		app.save();
	}
		
	return 0;
}
