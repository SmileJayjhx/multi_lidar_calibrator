# Multi LiDAR Calibrator

作用: 提供了一个多激光雷达的联合标定策略, 同时支持2d-3d联合匹配, 方法为将3d激光雷达点云的z轴数据进行选择滤波, 对滤波后的数据进行投影, 之后与2d雷达的点云进行点云匹配, 这里采用的是ICP与GICP算法

参考资料: 

​	@[Autoware](https://github.com/AbangLZU/Autoware) [Multi LiDAR Calibrator](https://github.com/AbangLZU/Autoware/tree/master/ros/src/sensing/fusion/packages/multi_lidar_calibrator)

​	原文给出的是一个基于NDT的点云匹配机制, 从而实现多雷达的联合标定

## 使用方法

1. 下载与编译

```shell
mkdir catkin_ws
cd catkin_ws
git clone https://github.com/SmileJayjhx/multi_lidar_calibrator.git
cd multi_lidar_calibrator
catkin_make
```

2. 参数修改

   进入launch文件中修改各项参数, 匹配自己的雷达话题等

   注:

   ​	这里我的雷达是livox的, 输出话题的消息类型不是标准的pointcloud2, 因此最后添加了一个脚本, 将该话题的点云进行提取并转发

   ​	如果数据是不同时间录制的数据包, 则需要进行时间同步才可以触发回调函数, 在脚本中有样例可以修改

3. 启动节点

   ```shell
   roslaunch multi_lidar_calibrator multi_lidar_calibrator.launch
   ```

## 参数说明

Parameter| Type| Description
----------|-----|--------
`points_parent_src`|*String* |作为参考系的雷达话题
`points_child_src`|*String*|待校准的雷达话题
`voxel_size`|*double*|在3D-3D降采样体素大小 Default: 0.5 (3D-3D生效)
`ndt_epsilon`|*double*|当NDT连续的迭代改进小于或等于该值时，算法会停止进一步的迭代，认为已经找到了最优解 Default: 0.01
`ndt_step_size`|*double*|NDT每一步迭代时沿着搜索方向移动的最大距离 Default: 0.1
`ndt_resolution`|*double*|在处理和分析点云数据时，原始点云将被划分成边长为该值的立方体小区域，每个小区域内的点被视为一个整体进行处理Default: 1.0
`ndt_iterations`|*double*|NDT 最大迭代次数 Default: 400
`proj`|bool|是否将3D点云投影为2D Default: True
`min_z_value`|*double*|z轴滤波器的范围 [m]
`max_z_value`|*double*|z轴滤波器的范围 [m]
`max_iterations_`|int|ICP和GICP的最大迭代次数
`transformation_epsilon`|double|如果连续两次迭代的变换小于这个阈值，算法会停止，认为已找到最佳对齐
`euclidean_fitness_epsilon`|double|算法停止前必须达到的点云之间的欧几里得距离差异的最大值
`x`|*double*|粗略估计的相对位姿(待校准雷达相对于参考系) [m]
`y`|*double*|粗略估计的相对位姿(待校准雷达相对于参考系) [m]
`z`|*double*|粗略估计的相对位姿(待校准雷达相对于参考系) [m]
`roll`|*double*|粗略估计的相对位姿(待校准雷达相对于参考系) [rad]
`pitch`|*double*|粗略估计的相对位姿(待校准雷达相对于参考系) [rad]
`yaw`|*double*|粗略估计的相对位姿(待校准雷达相对于参考系) [rad]

## Output

在rviz中会有提示, 其中不同算法产生的点云颜色不同, 可以选择一个较好的进行数据收集