/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * multi_lidar_calibrator.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include "multi_lidar_calibrator.h"


void ROSMultiLidarCalibratorApp::PublishCloud(const ros::Publisher& in_publisher, 
											  pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header.frame_id = parent_frame_;
	in_publisher.publish(cloud_msg);
}

void ROSMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
    pcl::PointCloud<PointT>::Ptr in_parent_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr in_child_cloud(new pcl::PointCloud<PointT>);
	ROS_DEBUG("PointsCallback activated!");

	pcl::PointCloud<PointT>::Ptr child_filtered_cloud (new pcl::PointCloud<PointT>);

	pcl::fromROSMsg(*in_parent_cloud_msg, *in_parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *in_child_cloud);

	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

	// 如果是两个3d雷达, 可以考虑降采样

	pcl::PointCloud<PointT>::Ptr processed_cloud(new pcl::PointCloud<PointT>);
	if(proj_==true)
	{
		ROS_ERROR_ONCE("正在进行投影!");
		// TODO: 可以过滤base_link坐标系下的点(卡点: 延迟严重, 点云匹配无法收敛)
		// 获取lidar_frame到base_link之间的坐标转换
		// tf::StampedTransform transform;
		// try{
		// 	listener.waitForTransform(base_frame_, lidar_frame_, ros::Time(0), ros::Duration(0));
		// 	listener.lookupTransform(base_frame_, lidar_frame_, ros::Time(0), transform);
		// 	// 计算新的 Z 范围	
		// 	min_z_value_ += transform.getOrigin().z();
		// 	max_z_value_ += transform.getOrigin().z();
		// }
		// catch (tf::TransformException &ex) {
		// 	ROS_ERROR("Fail to get tf between %s and %s: %s", base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
		// }
		// 2d-3d时添加范围滤波, 将z轴特定范围内的点投影到2d平面
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(in_parent_cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(min_z_value_, max_z_value_);
		pass.filter(*child_filtered_cloud);
		for (const auto& point : child_filtered_cloud->points) {
			processed_cloud->points.push_back(PointT(point.x, point.y, 0));
		}

		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*processed_cloud, cloud_msg);
		cloud_msg.header.frame_id = parent_frame_;
		proj_cloud_publisher_.publish(cloud_msg);
	}else
	{
		DownsampleCloud(in_child_cloud, child_filtered_cloud, voxel_size_);
		processed_cloud = child_filtered_cloud;
	}

	// Initializing Normal Distributions Transform (NDT).
	{
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;

	ndt.setTransformationEpsilon(ndt_epsilon_);
	ndt.setStepSize(ndt_step_size_);
	ndt.setResolution(ndt_resolution_);

	ndt.setMaximumIterations(ndt_iterations_);

	ndt.setInputSource(processed_cloud);
	ndt.setInputTarget(in_parent_cloud);
		
	pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

	Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
	Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());

	Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	if(current_guess_ == Eigen::Matrix4f::Identity())
	{
		current_guess_ = init_guess_;
	}

	ndt.align(*output_cloud, current_guess_);

	std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
	          << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;

	std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud (*in_child_cloud, *output_cloud, ndt.getFinalTransformation());

	current_guess_ = ndt.getFinalTransformation();

	Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
	Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);
	std::cout << "This transformation can be replicated using:" << std::endl;
	std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
	          << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
	          << " /" << child_frame_ << " 10" << std::endl;

	std::cout << "Corresponding transformation matrix:" << std::endl
	          << std::endl << current_guess_ << std::endl << std::endl;

	PublishCloud(calibrated_cloud_publisher_, output_cloud);
	}
	
	
	// ICP 匹配
	{
	pcl::PointCloud<PointT>::Ptr icp_output_cloud(new pcl::PointCloud<PointT>);
	// Assuming initial_x_, initial_y_, initial_z_, initial_roll_, initial_pitch_, initial_yaw_ are defined
	Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
	Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());
	Eigen::Matrix4f initial_pose = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    // 示例：使用ICP进行点云匹配
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(processed_cloud);
    icp.setInputTarget(in_child_cloud);
	icp.setMaximumIterations(max_iterations_); // You can adjust the number of iterations
	icp.setTransformationEpsilon(transformation_epsilon_); // Adjust the convergence criteria
	icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_); // Adjust the fitness criteria
	icp.align(*icp_output_cloud, initial_pose);

	// 如果收敛则发布消息
	if (icp.hasConverged()) {
    std::cout << "ICP converged." << std::endl
              << "The score is " << icp.getFitnessScore() << std::endl;

	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	Eigen::Matrix3f rotation_matrix = transformation.block(0, 0, 3, 3);
	Eigen::Vector3f translation_vector = transformation.block(0, 3, 3, 1);
	std::cout << "==============================ICP===========================================" << std::endl;
	std::cout << "This transformation can be replicated using:" << std::endl;
	std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
          << " " << rotation_matrix.eulerAngles(2, 1, 0).transpose() << " /" << parent_frame_
          << " /" << child_frame_ << " 10" << std::endl;

	std::cout << "Corresponding transformation matrix:" << std::endl
          << std::endl << transformation << std::endl << std::endl;
	std::cout << "==============================END===========================================" << std::endl;
	PublishCloud(icp_cloud_publisher_, icp_output_cloud);

	} else {
    std::cout << "ICP did not converge." << std::endl;
	}
	
	}

	// GICP 匹配
	{
		pcl::PointCloud<PointT>::Ptr gicp_output_cloud(new pcl::PointCloud<PointT>);

		// Assuming initial_x_, initial_y_, initial_z_, initial_roll_, initial_pitch_, initial_yaw_ are defined
		Eigen::Translation3f init_translation(initial_x_, initial_y_, initial_z_);
		Eigen::AngleAxisf init_rotation_x(initial_roll_, Eigen::Vector3f::UnitX());
		Eigen::AngleAxisf init_rotation_y(initial_pitch_, Eigen::Vector3f::UnitY());
		Eigen::AngleAxisf init_rotation_z(initial_yaw_, Eigen::Vector3f::UnitZ());
		Eigen::Matrix4f initial_pose = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

		// Example: Point cloud matching using GICP
		pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
		gicp.setInputSource(processed_cloud);
		gicp.setInputTarget(in_child_cloud);
		gicp.setMaximumIterations(max_iterations_); // Adjust the number of iterations
		gicp.setTransformationEpsilon(transformation_epsilon_); // Adjust the convergence criteria
		gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_); // Adjust the fitness criteria
		// Set the initial pose and align
		gicp.align(*gicp_output_cloud, initial_pose);

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

			std::cout << "Corresponding transformation matrix:" << std::endl
					<< std::endl << transformation << std::endl << std::endl;
			std::cout << "==============================END==========================================="
					<< std::endl;

			PublishCloud(gicp_cloud_publisher_, gicp_output_cloud);
		} else {
			std::cout << "GICP did not converge." << std::endl;
		}

		
	}

}

void ROSMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
	pcl::VoxelGrid<PointT> voxelized;
	voxelized.setInputCloud(in_cloud_ptr);
	voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	voxelized.filter(*out_cloud_ptr);
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
	// 滤波范围
	in_private_handle.param<double>("min_z_value", min_z_value_, -1.0);
	in_private_handle.param<double>("max_z_value", max_z_value_, 2.0);

	// ICP与GICP配置
	in_private_handle.param<int>("max_iterations", max_iterations_, 100);
	in_private_handle.param<double>("transformation_epsilon", transformation_epsilon_, 1e-8);
	in_private_handle.param<double>("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.1);

	// 坐标系配置
	in_private_handle.param<std::string>("base_frame", base_frame_, "base_link");
	in_private_handle.param<std::string>("lidar_frame", lidar_frame_, "livox_frame");


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
	ROS_ERROR("Start!");
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