#!/usr/bin/python3

import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from homework2.msg import custom_msg

from math import radians, pi

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=True)
        
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist , queue_size=10)
        
        # getting specified parameters
        # self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        # self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        # self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        # self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        # self.epsilon = rospy.get_param("/controller/epsilon")

        # self.linear_speed = 0.5 # m/s
        self.linear_speed = rospy.get_param("~linear_speed") # m/s

        self.angular_speed = 0.5 # r/s  
        # self.goal_angle = 0 # rad
        self.stop_distance = 2 # m
        self.epsilon = 0.5

        # twist = Twist()
        # twist.linear.x = self.linear_speed
        # twist.angular.z = 0
        # self.cmd_publisher.publish(twist)
        # rospy.sleep(10)

        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 
         
        
    # checks whether there is an obstacle in front of the robot
    # or not
    # def laser_callback(self, msg: LaserScan):
    #     if msg.ranges[0] <= self.stop_distance:
    #         self.state = self.ROTATE
    
    
    # heading of the robot 
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        return yaw

    def angular_rotation(self, remaining):
        if remaining < -pi:
            return remaining + 2 * pi
        elif remaining > pi:
            return remaining - 2 * pi
        return remaining
    
    def run(self):
    
        while not rospy.is_shutdown():

            if self.state == self.GO:
            #     # print('hi3\n\n\n\n\n\n\n\n')
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
                # rospy.loginfo('runing...')
            
            rospy.sleep(1)
            # print('hi2\n\n\n\n\n\n\n\n')
            # rospy.loginfo('hi2\n\n\n\n\n\n\n\n')
            obs_controller = rospy.wait_for_message('closest_obstacle', custom_msg)
            
            # print(f'minimum distance to obstacle: {obs_controller.distance}')
            if obs_controller.distance < self.stop_distance or self.state == self.ROTATE:
                direction = False
                print("\n\ndistance", obs_controller.distance)
                print("rotatoin degree", obs_controller.direction)
                print('\n\n')
                self.state = self.ROTATE
                rospy.loginfo('\n\n\nCLOSE TO OBSTCLE!\n\n\n\n')
                twist = Twist()
                twist.linear.x = 0
                self.cmd_publisher.publish(Twist())
                rospy.sleep(1)
                # print(obs_controller.direction)
                # print(f'direction: {180 - obs_controller.direction}')
                # rospy.loginfo(f'Rotate {180 - obs_controller.direction} degree')
                if obs_controller.direction < 180:
                    direction = True
                else:
                    direction = False

                remaining = self.angular_rotation(radians(180-obs_controller.direction))
                prev_angle = self.get_heading()

                twist = Twist()
                if direction:
                    twist.angular.z = -self.angular_speed
                else:
                    twist.angular.z = self.angular_speed

                self.cmd_publisher.publish(twist)
                # rotation loop
                print(remaining)
                remaining = abs(remaining)
                while (remaining) >= self.epsilon:
                    current_angle = self.get_heading()
                    delta = abs(prev_angle - current_angle)
                    remaining -= (delta)
                    prev_angle = current_angle
                    print(remaining)
                
                self.cmd_publisher.publish(twist)

                self.state = self.GO
                rospy.sleep(1)


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()

