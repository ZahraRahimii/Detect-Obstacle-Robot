#!/usr/bin/python3

import rospy
import tf

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import math
from homework2.msg import custom_msg
# from math import radians, pi

class Sensor:
    
    def __init__(self) -> None:
        
        rospy.init_node("sensor" , anonymous=True)
        self.path = Path()

        self.ranges = []
        self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.msg_publisher = rospy.Publisher("/closest_obstacle" , custom_msg , queue_size=10)    

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, callback=self.odom_callback)
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)

        # rospy.loginfo('sensor\n\n\n\n\n\n\n\n')

        # self.minimum_distance_calculation()
    
    def odom_callback(self, msg: Odometry):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

  
    def laser_callback(self, laser_range : LaserScan):
        rospy.loginfo('sensor\n\n\n\n')
        # print('sensor\n\n\n\n\n\n\n\n')
        min_distance = math.inf
        min_index = 0
        for id, range in enumerate(laser_range.ranges):
            print(f"direction and distance = {id} and {range}")
            if range < min_distance:
                min_distance = range
                min_index = id
        # print('distance is equal to ', min_distance)
        # angle = min_index
        # rotating_angle = self.angular_rotation(radians(angle))
        msg = custom_msg()
        msg.distance = min_distance
        msg.direction = min_index
        # msg.distance = 0
        # msg.direction = 20
        self.msg_publisher.publish(msg)


if __name__ == "__main__":
    sensor = Sensor()
    
    rospy.spin()

