#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include "opencv2/imgproc/detail/distortion_model.hpp"

#define _NODE_NAME_ "image_publisher"
using namespace cv;


class ImageTalker
{
public:
	ImageTalker()
	{
	}
	
	void parseCameraInfo()
	{
		static bool instrinsics_parsed = false;
		camera_info_msg_.header.stamp = ros::Time::now();
		if (instrinsics_parsed)
			return;
		
		camera_info_msg_.distortion_model = distModel_;
		camera_info_msg_.height = imgSize_.height;
		camera_info_msg_.width = imgSize_.width;
		camera_info_msg_.header.frame_id = frame_id_;
		
		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 3; col++)
				camera_info_msg_.K[row * 3 + col] = new_camera_instrinsics_.at<double>(row, col);
		}

		for (int row = 0; row < 3; row++)
		{
			for (int col = 0; col < 4; col++)
			{
				if (col == 3)
					camera_info_msg_.P[row * 4 + col] = 0.0f;
				else
					camera_info_msg_.P[row * 4 + col] = new_camera_instrinsics_.at<double>(row, col);
			}
		}
		
		for (int row = 0; row < distortion_coefficients_.rows; row++)
		{
			for (int col = 0; col < distortion_coefficients_.cols; col++)
				camera_info_msg_.D.push_back(distortion_coefficients_.at<double>(row, col));
		}
		instrinsics_parsed = true;	
	}
	
	void cameraInfoSubscriberConnected(const ros::SingleSubscriberPublisher& pub)
	{
		ROS_INFO("[%s] CameraInfo get a new subscriber.", _NODE_NAME_);
		this->parseCameraInfo();
		pub.publish(camera_info_msg_);
	}
	
	void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private,int cameraId)
	{
		camera_id_ = cameraId;
		
		image_transport::ImageTransport it(nh);
		
		std::string calibration_file_path;
		nh_private.param<std::string>("calibration_file_path",calibration_file_path,"None.yaml");
		nh_private.param<bool>("is_show_image",is_show_image_,false);
		nh_private.param<int>("frame_rate",frame_rate_,30);
		nh_private.param<float>("image_scale", imageScale_, 1.0);
		nh_private.param<bool>("cycle_pub_camera_info", cycle_pub_camera_info_, false);
		nh_private.param<std::string>("frame_id", frame_id_, "camera");
		
		if(!ros::param::get("~image_resolution",imageResolution_))
		{
			imageResolution_[0] = 640;
			imageResolution_[1] = 480;
		}
	
		ROS_INFO("[%s] calibration file: %s",_NODE_NAME_, calibration_file_path.c_str());
		
		if(!Loadintrinsics(calibration_file_path))
		{
			pub_ = it.advertise("/image_raw", 1);
			is_rectify_ = false;
		}
		else
		{
			pub_ = it.advertise("/image_rectified", 1);
			
			//获取摄像头的新内参，当图像校正后将会出现黑边，若通过裁剪使黑边消失，输出图像对应的相机内参变发生了变化
			//该函数最后一个参数为视场缩放因子，为1.0时视场大小不变，小于1时缩放视场
			new_camera_instrinsics_ = getOptimalNewCameraMatrix(camera_instrinsics_,distortion_coefficients_,imgSize_,0.0);
			
//			std::cout << camera_instrinsics_ << std::endl;
//			std::cout << new_camera_instrinsics_ << std::endl;
			
			is_rectify_ = true;
		}
		if(cycle_pub_camera_info_)
			camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);
		else
			camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1, boost::bind(&ImageTalker::cameraInfoSubscriberConnected, this, _1));
	}
	
	void run()
	{
		cv::VideoCapture cap(camera_id_);
		
		if(!cap.isOpened())
		{
			ROS_ERROR("can not open video device\n");
			return;
		}
		
		cap.set(CV_CAP_PROP_FPS, frame_rate_);
		cap.set(CV_CAP_PROP_FRAME_WIDTH, imageResolution_[0]);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, imageResolution_[1]);
		
		int w = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		int h = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		
		ROS_INFO("[%s] Expect image resolution: %d*%d", _NODE_NAME_,  imageResolution_[0],imageResolution_[1]);
		
		if(w != imageResolution_[0] || h != imageResolution_[1])
			ROS_ERROR("[%s] Not support the resolution: %d*%d",_NODE_NAME_, imageResolution_[0],imageResolution_[1]);
			
		ROS_INFO("[%s] Set image resolution:  %d*%d", _NODE_NAME_, w,h);
		
		cv::Mat frame, src;
		sensor_msgs::ImagePtr msg;
		
		cv::Size target_size(int(imageResolution_[0]*imageScale_), int(imageResolution_[1]*imageScale_));
		
		ros::Rate loop_rate(frame_rate_);
		
		while (ros::ok())
		{
			cap >> frame;
			
		
			if(frame.empty())
			{
				loop_rate.sleep();
				continue;
			}
			
			if(is_rectify_)
			{
				cv::undistort(frame, src, camera_instrinsics_, distortion_coefficients_,new_camera_instrinsics_);
				if(imageScale_!=1.0)
					cv::resize(src, src, target_size);
					
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
				msg->header.frame_id = frame_id_;
				if(is_show_image_) 
				{
					cv::namedWindow("image_rectified",cv::WINDOW_NORMAL); 
					cv::imshow("image_rectified",src); cv::waitKey(1);
				}
			}
			else
			{
				if(imageScale_!=1.0)
					cv::resize(frame, frame, target_size);
				
				
				//cv::flip(frame,frame,-1); //将摄像头图像翻转一次再发布
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
				msg->header.frame_id = frame_id_;
				if(is_show_image_) 
				{
					cv::namedWindow("image_raw",cv::WINDOW_NORMAL); 
					cv::imshow("image_raw",frame); cv::waitKey(1);
				}
			}
			pub_.publish(msg);
			if(cycle_pub_camera_info_)
			{
				this->parseCameraInfo();
				camera_info_pub_.publish(camera_info_msg_);
			}
			
			loop_rate.sleep();
		}
	}
	
	bool Loadintrinsics(const std::string &calibration_file_path)
	{
		if (calibration_file_path.empty())
		{
			ROS_ERROR("[%s] missing calibration file path", _NODE_NAME_);
			return false;
		}

		cv::FileStorage fs(calibration_file_path, cv::FileStorage::READ);

		if (!fs.isOpened())
		{
			ROS_INFO("[%s] cannot open calibration file %s", _NODE_NAME_, calibration_file_path.c_str());
	 		return false;
		}

		camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
		distortion_coefficients_ = cv::Mat(1, 5, CV_64F);

		cv::Mat dis_tmp;
		
		fs["CameraMat"] >> camera_instrinsics_;
		fs["DistCoeff"] >> dis_tmp;
		fs["ImageSize"] >> imgSize_;
		fs["DistModel"] >> distModel_;
		
		for (int col = 0; col < 5; col++)
		{
			distortion_coefficients_.at<double>(col) = dis_tmp.at<double>(col);
		}
		fs.release();	//释放
		return true;
	}
	
private:
	bool is_rectify_;
	image_transport::Publisher pub_ ;
	ros::Publisher camera_info_pub_;
	ros::Timer timer_;
	bool cycle_pub_camera_info_;
	std::string frame_id_; 
	
	cv::Mat new_camera_instrinsics_;
	cv::Mat camera_instrinsics_;
	cv::Mat distortion_coefficients_;
	std::string distModel_;
	sensor_msgs::CameraInfo camera_info_msg_;
	cv::Size imgSize_;
	float imageScale_;
	std::vector<int> imageResolution_;
	int camera_id_;
	int frame_rate_;
	bool is_show_image_;
};


int main(int argc, char** argv)
{
	if(argc <2) 
	{
		ROS_ERROR("argv[1]: camera number is empty\n");
		return 1;
	}

	ros::init(argc, argv, _NODE_NAME_);
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	int camera_id = atoi(argv[1]);
	
	ImageTalker image_talker;
	image_talker.init(nh,nh_private,camera_id);
	image_talker.run();
	return 0;
}
