#include "multi_lidar_calibrator.h"

/**
 * @brief 将点云通过指定的发布者发布出去
 * @param in_publisher 发布者名称
 * @param in_cloud_to_publish_ptr 点云指针
*/
void ROSMultiLidarCalibratorApp::PublishCloud(const ros::Publisher& in_publisher, 
											  pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header.frame_id = parent_frame_;
	in_publisher.publish(cloud_msg);
}

/**
 * @brief 获取base_frame_(目标点云的frame_id)以及lidar_frame_之间的tf变换
 * @return transform_  通过引用传出, 表示两个坐标系之间的变换关系
*/
void ROSMultiLidarCalibratorApp::TransformConverter()
{
	ROS_WARN("TransformConverter has been called!");
	try{
		listener.waitForTransform(base_frame_, lidar_frame_, ros::Time(0), ros::Duration(0));
		listener.lookupTransform(base_frame_, lidar_frame_, ros::Time(0), transform_);
		ROS_WARN("z between %s and %s: %f", base_frame_.c_str(), lidar_frame_.c_str(), transform_.getOrigin().z());
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("Fail to get tf between %s and %s: %s", base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
	}
}

/**
 * @brief 接收两个点云的消息进行配准, 注: 需要完成两个消息类型的同步(在跑离线包时注意修正时间)
 * @param in_parent_cloud_msg target点云的消息
 * @param in_child_cloud_msg source点云的消息
*/
void ROSMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
	ROS_DEBUG("PointsCallback activated!");
	// 初始化点云数据
	pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr in_child_cloud(new pcl::PointCloud<PointT>);
	pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *in_child_cloud);
	// 获取frame_id
	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

	ROS_INFO_ONCE("正在进行投影!");

	pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr parent_filtered_cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr processed_cloud(new pcl::PointCloud<PointT>);

	// parent 点云的处理, 处理后的点云为processed_cloud 发布到/proj_cloud
	{
		// 范围滤波	
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(in_parent_cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(min_z_value_, max_z_value_);
		ROS_ERROR("min_z_value_: %f max_z_value_: %f",min_z_value_,max_z_value_);
		pass.filter(*temp_cloud);
		parent_filtered_cloud = temp_cloud;
		// 检查处理后的点云是否为空
		if (parent_filtered_cloud->empty()) {
			ROS_ERROR("After PassThrough: Point cloud is empty!");
			return;
		}

		// 体素滤波
		pcl::VoxelGrid<PointT> voxelized;
		voxelized.setInputCloud(parent_filtered_cloud);
		voxelized.setLeafSize((float)voxel_size_, (float)voxel_size_, (float)voxel_size_);
		voxelized.filter(*temp_cloud);
		parent_filtered_cloud = temp_cloud;

		// 检查处理后的点云是否为空
		if (parent_filtered_cloud->empty()) {
			ROS_ERROR("After VoxelGrid: Point cloud is empty!");
			return;
		}

		// 移除统计离群值
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(parent_filtered_cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(0.9);
		sor.filter(*temp_cloud);
		parent_filtered_cloud = temp_cloud;
		if (parent_filtered_cloud->empty()) {
			ROS_ERROR("After StatisticalOutlierRemoval: Point cloud is empty!");
			return;
		}

		// 移除半径离群值
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
		ror.setInputCloud(parent_filtered_cloud);
		ror.setRadiusSearch(0.1);
		ror.setMinNeighborsInRadius(3);
		ror.filter(*temp_cloud);
		parent_filtered_cloud = temp_cloud;
		if (parent_filtered_cloud->empty()) {
			ROS_ERROR("After RadiusOutlierRemoval: Point cloud is empty!");
			return;
		}
		// 将完全处理好的parent点云放入processed_cloud, 并投影到2d平面
		for (const auto& point : parent_filtered_cloud->points) {
			processed_cloud->points.push_back(PointT(point.x, point.y, 0));
		}
		// 检查处理后的点云是否为空
		if (processed_cloud->empty()) {
			ROS_ERROR("After Projection: processed_cloud is empty! ");
			return;
		}
		// 发布处理后的parent点云信息到/proj_cloud
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*processed_cloud, cloud_msg);
		cloud_msg.header.frame_id = parent_frame_;
		proj_cloud_publisher_.publish(cloud_msg);
	}

	// child 点云的处理, 处理后的点云为in_child_cloud 发布到/vol_child_cloud
	{
		// 以预设体素滤波器size的1/8处理子点云
		pcl::PointCloud<PointT>::Ptr temp2_cloud(new pcl::PointCloud<PointT>);
		pcl::VoxelGrid<PointT> voxelized2;
		voxelized2.setInputCloud(in_child_cloud);
		voxelized2.setLeafSize((float)voxel_size_/2, (float)voxel_size_/2, (float)voxel_size_/2);
		voxelized2.filter(*temp2_cloud);
		in_child_cloud = temp2_cloud;

		// 发布处理后的子点云到/vol_child_cloud
		sensor_msgs::PointCloud2 cloud2_msg;
		pcl::toROSMsg(*in_child_cloud, cloud2_msg);
		cloud2_msg.header.frame_id = child_frame_;
		vol_child_publisher_.publish(cloud2_msg);
	}

	// Normal Distributions Transform (NDT). 
	// 根据处理后的点云为processed_cloud和in_child_cloud进行NDT
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	{
		// 初始化
		ndt.setTransformationEpsilon(ndt_epsilon_);
		ndt.setStepSize(ndt_step_size_);
		ndt.setResolution(ndt_resolution_);
		ndt.setMaximumIterations(ndt_iterations_);
		ndt.setInputSource(in_child_cloud);
		ndt.setInputTarget(processed_cloud);
		// 获取在launch文件中传入的初始位姿
		Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
		Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());
		Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
		// 第一次加载时的current_guess_为刚完成实例化后的单位阵, 将其修改为初始位姿
		if(current_guess_ == Eigen::Matrix4f::Identity())
		{
			current_guess_ = init_guess_;
		}
		// 应用NDT变换, 输出到output_cloud
		pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
		ndt.align(*output_cloud, current_guess_);

		std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
							<< " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;

		std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

		// 对in_child_cloud应用ndt转换, 输出到output_cloud
		pcl::transformPointCloud (*in_child_cloud, *output_cloud, ndt.getFinalTransformation());

		// 更新current_guess_并输出该值
		current_guess_ = ndt.getFinalTransformation();
		Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
		Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
		Eigen::Quaternionf quaternion(rotation_matrix);  // 从旋转矩阵创建四元数
		quaternion.normalize();  // 归一化四元数
		std::cout << "==============================NDT===========================================" << std::endl;
		std::cout << "TF2 Transformation (x, y, z, qx, qy, qz, qw): "
				<< translation_vector.transpose() << " "
				<< quaternion.x() << " " << quaternion.y() << " "
				<< quaternion.z() << " " << quaternion.w() << std::endl;
		std::cout << "迭代次数：" << ndt.getFinalNumIteration() << "预设最大次数: "<< ndt_iterations_ << std::endl;
    std::cout << "误差：" << ndt.getFitnessScore() << std::endl;
		std::cout << "==============================END===========================================" << std::endl << std::endl;;
		// 将ndt处理后的点云发布到/ntp_calib_cloud
		PublishCloud(calibrated_cloud_publisher_, output_cloud);
	}
	
	// ICP 匹配
	{
		pcl::PointCloud<PointT>::Ptr icp_output_cloud(new pcl::PointCloud<PointT>);
    // 示例：使用ICP进行点云匹配
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(in_child_cloud);
    icp.setInputTarget(processed_cloud);
		icp.setMaximumIterations(max_iterations_); 
		icp.setTransformationEpsilon(transformation_epsilon_); 
		icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_); 
		// 将ndt的结果当作icp的先验位姿
		Eigen::Matrix4f icp_init_pose = ndt.getFinalTransformation();
		icp.align(*icp_output_cloud, icp_init_pose);

		// 如果收敛则发布消息
		if (icp.hasConverged()) {
		Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
		Eigen::Matrix3f rotation_matrix = icp_transformation.block(0, 0, 3, 3);
		Eigen::Vector3f translation_vector = icp_transformation.block(0, 3, 3, 1);
		std::cout << "==============================ICP===========================================" << std::endl;
		std::cout << "This transformation can be replicated using:" << std::endl;
		std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
							<< " " << rotation_matrix.eulerAngles(2, 1, 0).transpose() << " /" << parent_frame_
							<< " /" << child_frame_ << " 10" << std::endl;
		std::cout << "迭代次数：" << icp.nr_iterations_ << "预设最大次数: "<< max_iterations_ << std::endl;
    std::cout << "误差：" << icp.getFitnessScore() << std::endl;
		std::cout << "Corresponding transformation matrix:" << std::endl
							<< icp_transformation << std::endl;
		std::cout << "==============================END===========================================" << std::endl << std::endl;
		PublishCloud(icp_cloud_publisher_, icp_output_cloud);

		} else std::cout << "ICP did not converge." << std::endl;
	}

	// GICP 匹配
	{
		pcl::PointCloud<PointT>::Ptr gicp_output_cloud(new pcl::PointCloud<PointT>);

		// Example: Point cloud matching using GICP
		pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
		gicp.setInputSource(in_child_cloud);
		gicp.setInputTarget(processed_cloud);
		gicp.setMaximumIterations(max_iterations_); 
		gicp.setTransformationEpsilon(transformation_epsilon_); 
		gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_); 
		// Set the initial pose and align
		gicp.align(*gicp_output_cloud, current_guess_);

		if (gicp.hasConverged()) {
			std::cout << "GICP converged." << std::endl
					<< "The score is " << gicp.getFitnessScore() << std::endl;

			Eigen::Matrix4f transformation = gicp.getFinalTransformation();
			Eigen::Matrix3f rotation_matrix = transformation.block(0, 0, 3, 3);
			Eigen::Vector3f translation_vector = transformation.block(0, 3, 3, 1);
			std::cout << "==============================GICP==========================================="
					<< std::endl;
			std::cout << "This transformation can be replicated using:" << std::endl;
			std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
					<< " " << rotation_matrix.eulerAngles(2, 1, 0).transpose() << " /" << parent_frame_
					<< " /" << child_frame_ << " 10" << std::endl;
			std::cout << "迭代次数：" << gicp.nr_iterations_ << "预设最大次数: "<< max_iterations_ << std::endl;
    	std::cout << "误差：" << gicp.getFitnessScore() << std::endl;
			std::cout << "Corresponding transformation matrix:" << std::endl
					<< std::endl << transformation << std::endl << std::endl;
			std::cout << "==============================END==========================================="
					<< std::endl;

			PublishCloud(gicp_cloud_publisher_, gicp_output_cloud);
		} else {
			std::cout << "GICP did not converge." << std::endl;
			ROS_ERROR("GICP did not converge");
		}

		
	}

}

void ROSMultiLidarCalibratorApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_parent_topic_str, points_child_topic_str;
	std::string calibrated_points_topic_str = "/ntp_calib_cloud";
	in_private_handle.param<std::string>("base_frame", base_frame_, "base_link");
	in_private_handle.param<std::string>("lidar_frame", lidar_frame_, "livox_frame");

	in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
	ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());

	in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
	ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());

	in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
	ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, voxel_size_);

	in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
	ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, ndt_epsilon_);

	in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
	ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

	in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
	ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

	in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
	ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

	in_private_handle.param<double>("x", initial_x_, 0.0);
	in_private_handle.param<double>("y", initial_y_, 0.0);
	in_private_handle.param<double>("z", initial_z_, 0.0);
	in_private_handle.param<double>("roll", initial_roll_, 0.0);
	in_private_handle.param<double>("pitch", initial_pitch_, 0.0);
	in_private_handle.param<double>("yaw", initial_yaw_, 0.0);

	in_private_handle.param<bool>("proj", proj_, false);

	// 坐标系配置
	in_private_handle.param<std::string>("base_frame", base_frame_, "lidar");
	in_private_handle.param<std::string>("lidar_frame", lidar_frame_, "livox_frame");
	// 滤波范围
	in_private_handle.param<double>("min_z_value", min_z_value_, -0.5);
	in_private_handle.param<double>("max_z_value", max_z_value_, 0.5);
	// 添加延时, 等待坐标系建立
	ros::Duration(0.5).sleep();
	// 获取base_link到lidar坐标系的转换关系, 并修正雷达过滤范围
	TransformConverter();
	min_z_value_ += transform_.getOrigin().z();
	max_z_value_ += transform_.getOrigin().z();

	// ICP与GICP配置
	in_private_handle.param<int>("max_iterations", max_iterations_, 100);
	in_private_handle.param<double>("transformation_epsilon", transformation_epsilon_, 1e-8);
	in_private_handle.param<double>("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.1);

	ROS_INFO("[%s] Initialization Transform x: %.2f y: %.2f z: %.2f roll: %.2f pitch: %.2f yaw: %.2f", __APP_NAME__,
	         initial_x_, initial_y_, initial_z_,
	         initial_roll_, initial_pitch_, initial_yaw_);
	ROS_INFO("[%s] ICP Config max_iterations: %d transformation_epsilon: %.2f euclidean_fitness_epsilon: %.2f", __APP_NAME__,
	         max_iterations_, transformation_epsilon_, euclidean_fitness_epsilon_);
	
	// 订阅者与同步器的配置
	cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                     points_parent_topic_str, 10);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

	cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                        points_child_topic_str, 10);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

	calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(calibrated_points_topic_str, 1);
	icp_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/icp_calib_cloud", 1);
	gicp_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/gicp_calib_cloud", 1);
	proj_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/proj_cloud", 1);
	vol_child_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/vol_child_cloud", 1);
	ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, calibrated_points_topic_str.c_str());

	cloud_synchronizer_ =
			new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
			                                               *cloud_parent_subscriber_,
			                                               *cloud_child_subscriber_);
	cloud_synchronizer_->registerCallback(boost::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this, _1, _2));
}


void ROSMultiLidarCalibratorApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	InitializeROSIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END",__APP_NAME__);
}

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
{
	//initialpose_quaternion_ = tf::Quaternion::getIdentity();
	current_guess_ = Eigen::Matrix4f::Identity();
}

