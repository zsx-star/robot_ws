
tf::listener
void transformPose(const std::string &target_frame, const geometry_msgs::PoseStamped &stamped_in, geometry_msgs::PoseStamped &stamped_out) const
姿态转换：将 stamped_in在自身frame_id下转换到 target_frame下的姿态 stamped_out.


T& tf2_ros::BufferInterface::transform (const T & in, T & out, const std::string & target_frame, ros::Duration timeout = ros::Duration(0.0) ) const
姿态转换：将 in在自身frame_id 转换的 target_frame 下的姿态 out

tf2::convert(const tf2::Quaternion& q, geometry_msgs::Quaternion& ori);
类型转换，tf2::Quaternion -> geometry_msgs::Quaternion
tf2::convert 支持多种tf类型数据 与ros消息进行转换（ Matching toMsg and from Msg conversion functions need to exist.）

geometry_msgs::TransformStamped tf2_ros::Buffer::lookupTransform(
		const std::string &  	target_frame,
		const ros::Time &  	target_time,
		const std::string &  	source_frame,
		const ros::Time &  	source_time,
		const std::string &  	fixed_frame,
		const ros::Duration  	timeout 
	) const virtual

在固定坐标系 fixed_frame下,source_time时刻source_frame相对于 target_time时刻 target_frame的变换。

通过rviz对机器人进行2D pose Estimate后发布一个预估初始位置，amcl订阅后考虑从预测位置发送，到预测位置接收这一时间段机器人的运动，并进行了补偿。
利用预估位置消息的时间与处理预估位置的时刻 通过tf工具判断两个时刻机器人的位移
