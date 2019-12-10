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

bool zero_orientation_set = false; 

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}


struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 		stcAngle;
struct SMag 		stcMag;
struct SDStatus 	stcDStatus;
struct SPress 		stcPress;
struct SLonLat 		stcLonLat;
struct SGPSV 		stcGPSV;
struct SQua             stcQua;

//convert serial data to jy901 data
bool CopeSerialData(std::string str_in)
{
    unsigned int str_length = str_in.size();
    static unsigned char chrTemp[2000];
    static unsigned char ucRxCnt = 0;
    static unsigned int usRxLength = 0;

    memcpy(chrTemp,str_in.data(),str_length);
    usRxLength += str_length;
    bool success = false;
    while (usRxLength >= 11)
    {
        if (chrTemp[0] != 0x55)
        {
            usRxLength--;
            memcpy(&chrTemp[0],&chrTemp[1],usRxLength);
            continue;
        }
        switch(chrTemp[1])
        {
            case 0x50:	memcpy(&stcTime,&chrTemp[2],8);break;
            case 0x51:	memcpy(&stcAcc,&chrTemp[2],8);break;
            case 0x52:	memcpy(&stcGyro,&chrTemp[2],8);break;
            case 0x53:	memcpy(&stcAngle,&chrTemp[2],8);  success = true; break;
            case 0x54:	memcpy(&stcMag,&chrTemp[2],8);break;
            case 0x55:	memcpy(&stcDStatus,&chrTemp[2],8);break;
            case 0x56:	memcpy(&stcPress,&chrTemp[2],8);break;
            case 0x57:	memcpy(&stcLonLat,&chrTemp[2],8);break;
            case 0x58:	memcpy(&stcGPSV,&chrTemp[2],8);break;
            case 0x59:  memcpy(&stcQua,&chrTemp[2],8);break;
            default: break;
        }
        usRxLength -= 11;
        memcpy(&chrTemp[0],&chrTemp[11],usRxLength);
    }
    return success;
}

//EulerToQuaternion, euler in rad
Eigen::Quaterniond euler2Quaternion(const double roll,  const double yaw, const double pitch)
{
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
	Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
	return q;
}


int main(int argc, char** argv)
{
  serial::Serial ser;
  std::string port;
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  uint8_t last_received_message_number;
  bool received_message = false;
  int data_packet_start;

  Eigen::Quaterniond orientation;
  Eigen::Quaterniond zero_orientation;

  ros::init(argc, argv, "mpu6050_serial_to_imu_node");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
  private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
  private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
  private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.0);

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);
  ros::Publisher imu_temperature_pub = nh.advertise<sensor_msgs::Temperature>("temperature", 50);
  ros::ServiceServer service = nh.advertiseService("set_zero_orientation", set_zero_orientation);

  ros::Rate r(200); // 200 hz

  sensor_msgs::Imu imu;

  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;

  sensor_msgs::Temperature temperature_msg;
  temperature_msg.variance = 0;

  static tf::TransformBroadcaster tf_br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0,0,0));

  std::string input;
  std::string read;

  while(ros::ok())
  {
    try
    {
      if (ser.isOpen())
      {
        // read string from serial device
        if(ser.available())
        {
        	if(!CopeSerialData(ser.read(ser.available())))
        	{
        		r.sleep();
        		continue;
        	}
        		
        	double roll = -1.0* stcAngle.Angle[1]/32768*M_PI;
        	double yaw = 1.0* stcAngle.Angle[2]/32768*M_PI;
        	double pitch = 1.0* stcAngle.Angle[0]/32768*M_PI;
            Eigen::Quaterniond quat = euler2Quaternion(roll, yaw, pitch);

            if (!zero_orientation_set)
            {
              zero_orientation = quat;
              zero_orientation_set = true;
            }

            //http://answers.ros.org/question/10124/relative-rotation-between-two-quaternions/
            Eigen::Quaterniond differential_rotation = zero_orientation.inverse() * quat;

                // calculate measurement time
            ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

            // publish imu message
            imu.header.stamp = measurement_time;
            imu.header.frame_id = frame_id;

            //quaternionTFToMsg(differential_rotation, imu.orientation);

            imu.orientation.w = differential_rotation.w();
            imu.orientation.x = differential_rotation.x();
            imu.orientation.y = differential_rotation.y();
            imu.orientation.z = differential_rotation.z();
            imu.linear_acceleration.x = (float)stcAcc.a[0]/32768*16.0;
            imu.linear_acceleration.y = (float)stcAcc.a[1]/32768*16.0;
            imu.linear_acceleration.z = (float)stcAcc.a[2]/32768*16.0;
            imu.angular_velocity.x = ((float)stcGyro.w[0]/32768*2000)/180.0*M_PI;
            imu.angular_velocity.y = ((float)stcGyro.w[1]/32768*2000)/180.0*M_PI;
            imu.angular_velocity.z = ((float)stcGyro.w[2]/32768*2000)/180.0*M_PI;

            imu_pub.publish(imu);

            // publish tf transform
            if (broadcast_tf)
            {
              transform.setRotation(tf::Quaternion(differential_rotation.x(),
              										differential_rotation.y(),
              										differential_rotation.z(),
              										differential_rotation.w()));
              tf_br.sendTransform(tf::StampedTransform(transform, measurement_time, tf_parent_frame_id, tf_frame_id));
            }
         
        }
      }
      else
      {
        // try and open the serial port
        try
        {
          ser.setPort(port);
          int baud_rate = private_node_handle.param<int>("baud_rate",115200);
          ser.setBaudrate(baud_rate);
          serial::Timeout to = serial::Timeout::simpleTimeout(1000);
          ser.setTimeout(to);
          ser.open();
        }
        catch (serial::IOException& e)
        {
          ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
          ros::Duration(5).sleep();
        }

        if(ser.isOpen())
        {
          ROS_INFO("Serial port %s initialized and opened. baudrate: %d",ser.getPort().c_str(),ser.getBaudrate());
        }
      }
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
      ser.close();
    }
    r.sleep();
  }
}
