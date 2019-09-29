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

#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

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
    
    
    uint8_t checknum;
});

class RobotDriver
{
public: 
	RobotDriver();
	~RobotDriver();
	void run();
private:
	bool initialize_params();
	bool initSerial(string& port_name,int baud_rate);
	bool initCan(string& port_name);
	void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd);
	void readSerialThread();
	
	uint8_t sumCheck(const uint8_t* buffer, size_t len);
	void distributeMsg(const uint8_t* buffer, size_t len);
	void bufferIncomingData(const uint8_t * buffer, size_t len);
	void handleEncoderMsg();
	
private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_private;
	ros::Subscriber mSubCmd;
	ros::Publisher mPubOdom;

	Can2serial *mCan;
	serial::Serial *mSerial;
	std::string mSerialPortName;
	bool mUseSerial;
	
	uint8_t * const mPkgBuffer;
	
	const encoderMsg_t *const mEncoderMsg;
	const imuMsg_t     *const mImuMsg;

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
	if(cmd->angular.z < 0)
		cmdBuf[9] |= 0x01;
	
//		for(int i=0; i<10; ++i)
//			printf("%x ",cmdBuf[i]);
//		printf("\n");
	mSerial->write(cmdBuf,10);
}

void RobotDriver::run()
{
	initialize_params();
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
	if(0x01 == pkg_id) //encoder
		handleEncoderMsg();
}

void RobotDriver::handleEncoderMsg()
{
	uint16_t encoder_A = mEncoderMsg->encoderA_val;
	uint16_t encoder_B = mEncoderMsg->encoderB_val;
	uint16_t encoder_C = mEncoderMsg->encoderC_val;
	
	
  rev_left_ = buffer_data[5] * 256 + buffer_data[6];
  rev_right_ = buffer_data[7] * 256 + buffer_data[8];

  cal_pulse(cur_left_, rev_left_, delta_left_);
  cal_pulse(cur_right_, rev_right_, delta_right_);

  ROS_DEBUG_STREAM("receive -> left: " << delta_left_ << "(" << rev_left_ << ")" << "; right: " << delta_right_ << "(" << rev_right_ << ")");

  now_ = ros::Time::now();
  if (start_flag_){
    accumulation_x_ = accumulation_y_ = accumulation_th_ = 0.0;
    last_time_ = now_;
    start_flag_ = false;
    return;
  }
  delta_time_ = (now_ - last_time_).toSec();
  if (delta_time_ >= (0.5 / control_rate_)){
    double model_param;
    if (delta_right_ <= delta_left_){
      model_param = model_param_cw_;
    }else{
      model_param = model_param_acw_;
    }
    double delta_theta = (delta_right_ - delta_left_)/ (pulse_per_cycle_ * pid_rate_ * model_param);
    double v_theta = delta_theta / delta_time_;

    double delta_dis = (delta_right_ + delta_left_) / (pulse_per_cycle_ * pid_rate_ * 2.0);
    double v_dis = delta_dis / delta_time_;

    double delta_x, delta_y;
    if (delta_theta == 0){
      delta_x = delta_dis;
      delta_y = 0.0;
    }else{
      delta_x = delta_dis * (sin(delta_theta) / delta_theta);
      delta_y = delta_dis * ( (1 - cos(delta_theta)) / delta_theta );
    }

    accumulation_x_ += (cos(accumulation_th_) * delta_x - sin(accumulation_th_) * delta_y);
    accumulation_y_ += (sin(accumulation_th_) * delta_x + cos(accumulation_th_) * delta_y);
    accumulation_th_ += delta_theta;
    
    if(odom_increment_)
    {
    	accumulation_x_+= odom_increment_->x;
    	accumulation_y_+= odom_increment_->y;
    	accumulation_th_+= odom_increment_->theta;
    	odom_increment_.reset(NULL);
    }

    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = odom_frame_;
    transformStamped_.child_frame_id = base_frame_;
    transformStamped_.transform.translation.x = accumulation_x_;
    transformStamped_.transform.translation.y = accumulation_y_;
    transformStamped_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, accumulation_th_);
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
   

    br_.sendTransform(transformStamped_);

    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = base_frame_;
    odom_.header.stamp = now_;
    odom_.pose.pose.position.x = accumulation_x_;
    odom_.pose.pose.position.y = accumulation_y_;
    odom_.pose.pose.position.z = 0;
    odom_.pose.pose.orientation.x = q.getX();
    odom_.pose.pose.orientation.y = q.getY();
    odom_.pose.pose.orientation.z = q.getZ();
    odom_.pose.pose.orientation.w = q.getW();
    odom_.twist.twist.linear.x = v_dis;
    odom_.twist.twist.linear.y = 0;
    odom_.twist.twist.angular.z = v_theta;

    odom_pub_.publish(odom_);

    ROS_DEBUG_STREAM("accumulation_x: " << accumulation_x_ << "; accumulation_y: " << accumulation_y_ <<"; accumulation_th: " << accumulation_th_);
  }
  last_time_ = now_;
}

uint8_t RobotDriver::sumCheck(const uint8_t* buffer, size_t len)
{
	uint8_t sum = 0;
	for(size_t i=0; i<len; ++i)
		sum += buffer[i];
	return sum;
}

bool RobotDriver::initialize_params()
{
	mSerialPortName = nh_private.param<std::string>("port_name","/dev/ttyUSB0");
	mUseSerial = nh_private.param<bool>("use_serial",true);
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
	
	//send 10 bytes random data to enable the robot
	for(uint8_t i=0; i<10; ++i)
		mSerial->write(&i,1);
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

