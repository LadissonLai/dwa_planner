#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

class MapRepublisher:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('local_map_republisher', anonymous=True)
        rospy.loginfo("Local Map Republisher Node Initialized")
        
        # 订阅原始 /map 话题
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # 创建新的发布者，发布到 /map_1hz 话题
        self.map_pub = rospy.Publisher('/local_map', OccupancyGrid, queue_size=10)
        
        # 设置发布频率为 1Hz
        self.rate = rospy.Rate(3)  # 1Hz
        self.latest_map = None
        
    def map_callback(self, msg):
        # 存储最新的地图消息
        self.latest_map = msg
        
    def republish(self):
        while not rospy.is_shutdown():
            if self.latest_map is not None:
                # 以 1Hz 频率发布最新的地图消息
                self.map_pub.publish(self.latest_map)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        republisher = MapRepublisher()
        republisher.republish()
    except rospy.ROSInterruptException:
        pass