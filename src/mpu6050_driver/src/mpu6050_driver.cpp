#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>
#include "JY901.h"

#define ACCELERATE_RANGE 2
#define ANGLE_RATE_RANGE 500

//EulerToQuaternion, euler in rad
Eigen::Quaterniond euler2Quaternion(const double roll,  const double yaw, const double pitch)
{
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
	Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
	return q;
}

class ImuDriver
{
public:
	ImuDriver();
	bool init();
	void run();
private:
	bool checkSum(const unsigned char* buf, int len);
	void dataProcess(const uint8_t* buf, int len);
	void pkgParse(const uint8_t* pkg, int len);
	void publish_msg();
	
private:
	struct STime		stcTime;
	struct SAcc 		stcAcc;
	struct SGyro 		stcGyro;
	struct SAngle 		stcAngle;
	struct SMag 		stcMag;
	struct SDStatus 	stcDStatus;
	struct SPress 		stcPress;
	struct SLonLat 		stcLonLat;
	struct SGPSV 		stcGPSV;
	struct SQua			stcQua;
	
	bool zero_orientation_set;
	
	 serial::Serial ser;
	std::string port;
	std::string tf_parent_frame_id;
	std::string frame_id;
	bool broadcast_tf;
	double linear_acceleration_stddev;
	double angular_velocity_stddev;
	double orientation_stddev;
	bool use_relative_angle;
	Eigen::Quaterniond orientation;
	Eigen::Quaterniond zero_orientation;
	ros::Publisher imu_pub ;
	ros::Publisher imu_temperature_pub;
	sensor_msgs::Imu imu;
	sensor_msgs::Temperature temperature_msg;
	tf::TransformBroadcaster tf_br;
	int baud_rate;
};

ImuDriver::ImuDriver()
{
	zero_orientation_set = false;
}

bool ImuDriver::init()
{
	ros::NodeHandle private_node_handle("~");
	private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
	private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
	private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
	private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
	private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
	private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
	private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);
	private_node_handle.param<int>("baud_rate",baud_rate, 115200);
	private_node_handle.param<bool>("use_relative_angle", use_relative_angle, false);

	ros::NodeHandle nh;
	imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);
	imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
	
	imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
	imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
	imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

	imu.angular_velocity_covariance[0] = angular_velocity_stddev;
	imu.angular_velocity_covariance[4] = angular_velocity_stddev;
	imu.angular_velocity_covariance[8] = angular_velocity_stddev;

	imu.orientation_covariance[0] = orientation_stddev;
	imu.orientation_covariance[4] = orientation_stddev;
	imu.orientation_covariance[8] = orientation_stddev;
	
	temperature_msg.variance = 0;
	return true;
}

void ImuDriver::run()
{
	const int MaxLen = 100;
	uint8_t *rawDataBuf = new uint8_t[MaxLen];//////////
	
	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		if (ser.isOpen())
		{
			int len = ser.read(rawDataBuf, MaxLen);
//			std::cout << len << std::endl;
			if(len <=0)
			{
				loop_rate.sleep();
				continue;
			}
			dataProcess(rawDataBuf, len);
		}
		else
		{
			try
			{
				ser.setPort(port);
				ser.setBaudrate(baud_rate);
				serial::Timeout to = serial::Timeout::simpleTimeout(1000);
				ser.setTimeout(to);
				ser.open();
			}
			catch (serial::IOException& e)
			{
				ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 1 seconds.");
				ros::Duration(1.0).sleep();
			}

			if(ser.isOpen())
				ROS_INFO("Serial port %s initialized and opened. baudrate: %d",ser.getPort().c_str(),ser.getBaudrate());
		}
		loop_rate.sleep();
	}
	delete [] rawDataBuf;
}

bool ImuDriver::checkSum(const unsigned char* buf, int len)
{
	uint8_t sum = 0;
	for(int i=0; i<len-1; ++i)
		sum += buf[i];
	if(sum == buf[len-1])
		return true;
	return false;
}


void ImuDriver::dataProcess(const uint8_t* buf, int len)
{
	static int pos = 0;
	static const int pkg_len = 11;
	static uint8_t pkg[pkg_len];
	
	for(int i=0; i<len; ++i)
	{
		if(pos == 0)
		{
			if(buf[i] == 0x55)
				pkg[pos++] = buf[i];
		}
		else if(pos ==1)
		{
			if(buf[i]>=0x50 && buf[i]<=0x59)
				pkg[pos++] = buf[i];
			else
				pos = 0;
		}
		else if(pos < pkg_len-1)
			pkg[pos++] = buf[i];
		else if(pos == pkg_len-1)
		{
			pkg[pos] = buf[i];
			pkgParse(pkg, pkg_len);
			pos = 0;
		}
	}
}

void ImuDriver::pkgParse(const uint8_t* pkg, int len)
{
//	for(int i=0; i<len; ++i)
//		std::cout << std::hex << int(pkg[i]) << "\t";
//	std::cout << std::endl;
	static uint8_t last_type = 0;
	if(!checkSum(pkg,len))
	{
		std::cout << "checkSum failed!!\n";
		return;
	}
	uint8_t type = pkg[1];
//	std::cout << std::hex << int(pkg[1]) << "\n";
	switch(type)
	{
		case 0x50:	memcpy(&stcTime,&pkg[2],8);break;
		case 0x51:	memcpy(&stcAcc,&pkg[2],8);break;
		case 0x52:	memcpy(&stcGyro,&pkg[2],8);break;
		case 0x53:	memcpy(&stcAngle,&pkg[2],8);break;
		case 0x54:	memcpy(&stcMag,&pkg[2],8);break;
		case 0x55:	memcpy(&stcDStatus,&pkg[2],8);break;
		case 0x56:	memcpy(&stcPress,&pkg[2],8);break;
		case 0x57:	memcpy(&stcLonLat,&pkg[2],8);break;
		case 0x58:	memcpy(&stcGPSV,&pkg[2],8);break;
		case 0x59:  memcpy(&stcQua,&pkg[2],8);break;
		default: break;
	}
	if(type < last_type)
		publish_msg();
	last_type = type;
}

void ImuDriver::publish_msg()
{

	double roll = -1.0* stcAngle.Angle[1]/32768*M_PI;
	double yaw = 1.0* stcAngle.Angle[2]/32768*M_PI;
	double pitch = 1.0* stcAngle.Angle[0]/32768*M_PI;
//	ROS_INFO("roll: %.2f\t yaw: %.2f\t pitch: %.2f", roll*180.0/M_PI, yaw*180.0/M_PI, pitch*180.0/M_PI);
	Eigen::Quaterniond quat = euler2Quaternion(roll, yaw, pitch);

	if (!zero_orientation_set)
	{
	  zero_orientation = quat;
	  zero_orientation_set = true;
	}

	if(use_relative_angle)
		quat = zero_orientation.inverse() * quat;
	
	ros::Time time = ros::Time::now();
	
	// publish imu message
	imu.header.stamp = time;
	imu.header.frame_id = frame_id;

	imu.orientation.w = quat.w();
	imu.orientation.x = quat.x();
	imu.orientation.y = quat.y();
	imu.orientation.z = quat.z();
	imu.linear_acceleration.x = (float)stcAcc.a[0]/32768*ACCELERATE_RANGE;
	imu.linear_acceleration.y = (float)stcAcc.a[1]/32768*ACCELERATE_RANGE;
	imu.linear_acceleration.z = (float)stcAcc.a[2]/32768*ACCELERATE_RANGE;
	imu.angular_velocity.x = ((float)stcGyro.w[0]/32768*ANGLE_RATE_RANGE)/180.0*M_PI;
	imu.angular_velocity.y = ((float)stcGyro.w[1]/32768*ANGLE_RATE_RANGE)/180.0*M_PI;
	imu.angular_velocity.z = ((float)stcGyro.w[2]/32768*ANGLE_RATE_RANGE)/180.0*M_PI;

	imu_pub.publish(imu);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0,0,0));
	
	if (broadcast_tf)
	{
		transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
		tf_br.sendTransform(tf::StampedTransform(transform, time, tf_parent_frame_id, frame_id));
	}
//	ROS_INFO("publish");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mpu6050_serial_to_imu_node");
	ImuDriver imu;
	if(imu.init())
		imu.run();
	return 0;
}
