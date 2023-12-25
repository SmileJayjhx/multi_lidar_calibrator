#!/usr/bin/env python
import rospy
import struct
from sensor_msgs.msg import PointCloud, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

class PointCloudConverter:
    def __init__(self):
        rospy.init_node('pointcloud_to_pointcloud2_converter', anonymous=True)
        # 获取参数，或使用默认值
        input_topic_pointcloud = rospy.get_param('~input_topic_pointcloud', '/lidar1/cloud')
        output_topic = rospy.get_param('~output_topic_pointcloud2', '/lidar1/cloud2')
        input_topic_pointcloud2 = rospy.get_param('~livox_input', '/livox/lidar')
        output_topic_pointcloud2 = rospy.get_param('~livox_output', '/lidar2/cloud2')

        rospy.Subscriber(input_topic_pointcloud, PointCloud, self.callback)
        self.pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)

        rospy.Subscriber(input_topic_pointcloud2, PointCloud2, self.callback_pointcloud2)
        self.pub_forward = rospy.Publisher(output_topic_pointcloud2, PointCloud2, queue_size=10)
        
        rospy.Subscriber("/lidar1/cloud", PointCloud, self.callback)
        self.pub = rospy.Publisher("/lidar1/cloud2", PointCloud2, queue_size=10)

        rospy.Subscriber("/livox/lidar", PointCloud2, self.callback_pointcloud2)
        self.pub_forward = rospy.Publisher("/lidar2/cloud2", PointCloud2, queue_size=10)

    def callback(self, data):
        cloud2 = self.convert_to_pointcloud2(data)
        cloud2.header.stamp = rospy.Time.now()
        self.pub.publish(cloud2)
    def callback_pointcloud2(self, data):
        # Simply forward the received PointCloud2 data
        data.header.stamp = rospy.Time.now()
        self.pub_forward.publish(data)
    def convert_to_pointcloud2(self, cloud):
        # Assume all channels have float32 data type
        fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        offset = 12
        for channel in cloud.channels:
            fields.append(PointField(name=channel.name, offset=offset, datatype=PointField.FLOAT32, count=1))
            offset += 4

        cloud2_data = []
        for i in range(len(cloud.points)):
            point = cloud.points[i]
            data = [point.x, point.y, point.z]
            for channel in cloud.channels:
                data.append(channel.values[i])
            cloud2_data.append(data)

        cloud2 = pc2.create_cloud(cloud.header, fields, cloud2_data)
        return cloud2

if __name__ == '__main__':
    converter = PointCloudConverter()
    rospy.spin()
