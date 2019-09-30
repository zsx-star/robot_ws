#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <can2serial/can2serial.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <cmath>

using std::string;
using std::cout;
using std::endl;

#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#define ENCODER_MSG_ID 0x01
#define IMU_MSG_ID 0x02

PACK(
struct encoderMsg_t {
    uint8_t header1;  
    uint8_t header2;  
    uint8_t pkg_len;
    uint8_t pkg_id;
    uint16_t encoderA_val;
    uint16_t encoderB_val;
    uint16_t encoderC_val;
    uint8_t checknum;
});

PACK(
struct imuMsg_t {
    uint8_t header1;  
    uint8_t header2;  
    uint8_t pkg_len;
    uint8_t pkg_id;
    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t gyro_z;
    uint16_t yaw;
    uint8_t checknum;
});

class RobotDriver
{
public: 
	RobotDriver();
	~RobotDriver();
	void run();
private:
	bool initializeParams();
	bool initSerial(string& port_name,int baud_rate);
	bool initCan(string& port_name);
	void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd);
	void readSerialThread();
	
	uint8_t sumCheck(const uint8_t* buffer, size_t len);
	void distributeMsg(const uint8_t* buffer, size_t len);
	void bufferIncomingData(const uint8_t * buffer, size_t len);
	uint16_t calculatePulse(uint16_t &last,const uint16_t &current);
	void handleEncoderMsg();
	void handleImuMsg();
	
private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	ros::Subscriber mSubCmd;
	ros::Publisher mPubOdom;
	
	tf2_ros::TransformBroadcaster   mTfBroadcaster;
	geometry_msgs::TransformStamped mTransformStamped;
	
	nav_msgs::Odometry mOdom;
	std::string mOdomFrameId, mBaseFrameId;

	Can2serial    * mCan;
	serial::Serial* mSerial;
	std::string     mSerialPortName;
	bool            mUseSerial;
	uint8_t * const mPkgBuffer;
	
	const encoderMsg_t *const mEncoderMsg;
	const imuMsg_t     *const mImuMsg;
	
	//robot params
	float mWheelDiameter;
	float mRotationRadius;
	int   mEncoderResolution;
	
	Eigen::Matrix3f mBase2wheelMatrix;
	Eigen::Matrix3f mWheel2baseMatrix;
	
	Eigen::Vector3f mPose; //x,y,theta
};

RobotDriver::RobotDriver():
	nh_private(ros::NodeHandle("~")),
	mPkgBuffer(new uint8_t[20]),
	mEncoderMsg((encoderMsg_t *)mPkgBuffer),
	mImuMsg((imuMsg_t *)mPkgBuffer)
{
}
RobotDriver::~RobotDriver()
{
	delete [] mPkgBuffer;
	if(!mSerial)
	{
		mSerial->close();
		delete mSerial;
		mSerial = NULL;
	}
	if(!mCan)
	{
		delete mCan;
		mCan = NULL;
	}
}

bool RobotDriver::initializeParams()
{
	mSerialPortName = nh_private.param<std::string>("port_name","/dev/ttyUSB0");
	mUseSerial = nh_private.param<bool>("use_serial",true);
	mWheelDiameter = nh_private.param<float>("wheel_diameter",0.1);
	mRotationRadius = nh_private.param<float>("rotation_radius",0.175);
	mEncoderResolution = nh_private.param<int>("encoder_resolution",1400);
	mBaseFrameId = nh_private.param<std::string>("base_frame","base_link");
	mOdomFrameId = nh_private.param<std::string>("odom_frame","odom");
	
	mBase2wheelMatrix << 1.0         , 0.0         , -mRotationRadius,
						 -sin(M_PI/6),  cos(M_PI/6), -mRotationRadius,
						 -cos(M_PI/3), -sin(M_PI/3), -mRotationRadius;
	mWheel2baseMatrix = mBase2wheelMatrix.inverse();
}

void RobotDriver::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
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
	if(cmd->angular.z > 0)
		cmdBuf[9] |= 0x01;
	
//		for(int i=0; i<10; ++i)
//			printf("%x ",cmdBuf[i]);
//		printf("\n");
	mSerial->write(cmdBuf,10);
}

void RobotDriver::run()
{
	initializeParams();
	mSubCmd = nh.subscribe("/cmd_vel",1,&RobotDriver::cmd_callback,this);
	mPubOdom = nh.advertise<nav_msgs::Odometry>("/odom",50);
	if(mUseSerial)
		initSerial(mSerialPortName, nh_private.param<int>("baud_rate",115200));
	else
		initCan(mSerialPortName);
	boost::thread parse_thread(boost::bind(&RobotDriver::readSerialThread, this));
}

void RobotDriver::readSerialThread()
{
	const int MaxReadNum = 50;
	uint8_t * const buffer = new uint8_t[MaxReadNum];

	size_t len;
	while(ros::ok())
	{
		try 
		{
			len = mSerial->read(buffer, MaxReadNum);
		}
		catch (std::exception &e) 
		{
			std::stringstream output;
			output << "Error reading from serial port: " << e.what();
			std::cout << output.str() <<std::endl;
			continue;
    	}
    	if(len==0) continue;

		bufferIncomingData(buffer, len);
	}
	
	delete [] buffer;
}

void RobotDriver::bufferIncomingData(const uint8_t * buffer, size_t len)
{
	static int pkg_buffer_index=0, pkg_len, pkg_type;
	for(size_t i=0; i<len; ++i)
	{
		if(0 == pkg_buffer_index)
		{
			if(0x55 == buffer[i])
				mPkgBuffer[pkg_buffer_index++] = buffer[i];
		}
		else if(1 == pkg_buffer_index)
		{
			if(0xAA == buffer[i])
				mPkgBuffer[pkg_buffer_index++] = buffer[i];
			else
				pkg_buffer_index = 0;
		}
		else if(2 == pkg_buffer_index)
			pkg_len = mPkgBuffer[pkg_buffer_index++] = buffer[i];
		else if(pkg_len+3 == pkg_buffer_index)
		{
			mPkgBuffer[pkg_buffer_index++] = buffer[i];//check_num
			distributeMsg(mPkgBuffer, pkg_buffer_index);
			pkg_buffer_index = 0;
		}
		else
			mPkgBuffer[pkg_buffer_index++] = buffer[i];
	}
}

void RobotDriver::distributeMsg(const uint8_t* buffer, size_t len)
{
	if(buffer[len-1] != sumCheck(buffer+2,len-3))
		return;
	uint8_t pkg_id = buffer[3];
	if(ENCODER_MSG_ID == pkg_id) //encoder
		handleEncoderMsg();
	else if(IMU_MSG_ID == pkg_id)
		handleImuMsg();
}

uint16_t RobotDriver::calculatePulse(uint16_t& last,const uint16_t &current)
{
	uint16_t delta;
	if (current > last)
		delta = (current - last) < (last - current + 65535) ? (current - last) : (current - last - 65535);
	else
		delta = (last - current) < (current - last + 65535) ? (current - last) : (current - last + 65535);

	last = current;
	return delta;
}

void RobotDriver::handleImuMsg()
{
	float accel_x = (1.0*mImuMsg->accel_x - 32768)/32768 * (9.81*2); //m/s2
	float accel_y = (1.0*mImuMsg->accel_y - 32768)/32768 * (9.81*2);
	float gyro_z  = (1.0*mImuMsg->gyro_z  - 32768)/32768 * 2000;     //deg/s
	float yaw     = (1.0*mImuMsg->yaw      -32768)/100;              //deg
	
	cout << accel_x  << "\t" << accel_y  << "\t" << gyro_z  << "\t" << yaw << endl;
}

void RobotDriver::handleEncoderMsg()
{
	static ros::Time last_time, current_time;
	static bool odom_start_flag = true;
	static uint16_t last_value_A=0, last_value_B=0, last_value_C=0;
	
	int16_t delta_A = calculatePulse(last_value_A, mEncoderMsg->encoderA_val);
	int16_t delta_B = calculatePulse(last_value_B, mEncoderMsg->encoderB_val);
	int16_t delta_C = calculatePulse(last_value_C, mEncoderMsg->encoderC_val);
	
//	std::cout << mEncoderMsg->encoderA_val << "\t" << mEncoderMsg->encoderB_val << "\t" << mEncoderMsg->encoderC_val << std::endl;
//	std::cout << delta_A << "\t" << delta_B << "\t" << delta_C << std::endl;
	
	current_time = ros::Time::now();
	
	if(odom_start_flag)
	{
		odom_start_flag = false;
		last_time = current_time;
		mPose = Eigen::Vector3f::Zero();
		return;
	}
	
	static float coff_inverse = M_PI * mWheelDiameter/mEncoderResolution;
	Eigen::Vector3f delta_wheel_dis;
	delta_wheel_dis[0] = delta_A * coff_inverse;
	delta_wheel_dis[1] = delta_B * coff_inverse;
	delta_wheel_dis[2] = delta_C * coff_inverse;
	
	Eigen::Vector3f delta_pose = mWheel2baseMatrix * delta_wheel_dis;
	Eigen::Vector3f speed = delta_pose/(current_time - last_time).toSec();
	mPose += delta_pose;
	
//	cout <<"1: "<<delta_wheel_dis.transpose() << endl;
//	cout <<"2: "<<delta_pose.transpose() << endl;
	
	if(mPose[2]> 2*M_PI)
		mPose[2] -= 2*M_PI;
	else if(mPose[2] < -2*M_PI)
		mPose[2] += 2*M_PI;
	
	//cout << mPose[0] << "\t" << mPose[1] << "\t" <<  mPose[2]*180.0/M_PI << endl;
	

	mTransformStamped.header.stamp = current_time;
	mTransformStamped.header.frame_id = mOdomFrameId;
	mTransformStamped.child_frame_id = mBaseFrameId;
	mTransformStamped.transform.translation.x = mPose[0];
	mTransformStamped.transform.translation.y = mPose[1];
	mTransformStamped.transform.translation.z = 0.0;
	tf::Quaternion q;
	q.setRPY(0, 0, mPose[2]);
	mTransformStamped.transform.rotation.x = q.x();
	mTransformStamped.transform.rotation.y = q.y();
	mTransformStamped.transform.rotation.z = q.z();
	mTransformStamped.transform.rotation.w = q.w();

	mTfBroadcaster.sendTransform(mTransformStamped);

	mOdom.header.frame_id = mOdomFrameId;
	mOdom.child_frame_id = mBaseFrameId;
	mOdom.header.stamp = current_time;
	mOdom.pose.pose.position.x = mPose[0];
	mOdom.pose.pose.position.y = mPose[1];
	mOdom.pose.pose.position.z = 0;
	mOdom.pose.pose.orientation.x = q.getX();
	mOdom.pose.pose.orientation.y = q.getY();
	mOdom.pose.pose.orientation.z = q.getZ();
	mOdom.pose.pose.orientation.w = q.getW();
	mOdom.twist.twist.linear.x = speed[0];
	mOdom.twist.twist.linear.y = speed[1];
	mOdom.twist.twist.angular.z = speed[2];
	mPubOdom.publish(mOdom);

	last_time = current_time;
}

uint8_t RobotDriver::sumCheck(const uint8_t* buffer, size_t len)
{
	uint8_t sum = 0;
	for(size_t i=0; i<len; ++i)
		sum += buffer[i];
	return sum;
}

bool RobotDriver::initCan(string& port_name)
{
	mCan = new Can2serial();
	
	if(!mCan->configPort(port_name))
	{
		ROS_ERROR("[%s] open %s failed!",ros::this_node::getName().c_str(), port_name.c_str());
		return false;
	}
	return true;
}

bool RobotDriver::initSerial(string& port_name,int baud_rate)
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
	return true;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"robot_driver_node");
	RobotDriver driver;
	driver.run();
	ros::spin();
	return 0;
}

