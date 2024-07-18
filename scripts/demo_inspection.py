#!/usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
import sys
#import serial
from rclpy.node import Node
from std_msgs.msg import Empty, Int32
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import time

example_group_number = 12

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""

def calculate_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def calculate_quaternion(yaw):
    z = math.sin(yaw * 0.5)
    w = math.cos(yaw * 0.5)
    return (0.0, 0.0, z, w)


class TestStudent(Node):

    # Constructor using number entered in console
    def __init__(self, team_number):
        super().__init__(f"test_student_{team_number}")
        self.publisher_ = self.create_publisher(Pose, f"team_{team_number}_pose", 10) # fstring lets you format a string in its statement
        self.other_publishers = self.create_publisher(Empty, "other_robot_stuff", 10)
        self.ready_publisher = self.create_publisher(Int32, f"team_{team_number}_ready", 10)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.odom_subscriber = self.create_subscription(Odometry,'odom', self.populate, 10)
        self.competition_start_subscription = self.create_subscription(
            Empty,
            'CompetitionStart',
            self.start_callback,
            10
        
        )
        self.navigator = BasicNavigator()
        self.start = False
        self.position_info = Pose()


    # Publish pose (you can do this at any rate you want)
    def timer_callback(self):
    
        ready_msg = Int32()
        ready_msg.data = example_group_number
        self.ready_publisher.publish(ready_msg)
    

        # If start has not been received
        if  not self.start:
            return

        # Publish pose message
        msg = Pose()

        msg.position.x = self.position_info.position.x
        msg.position.y = self.position_info.position.y
        msg.position.z = self.position_info.position.z

        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0 # this needs some mathematics!! and needs to be in the competition pose convention!
        msg.orientation.w = 0.0 #


        self.publisher_.publish(msg)
        
        # vel_msg = Twist()
        # vel_msg.linear.x = 1.0
        # self.vel_pub.publish(vel_msg)
        
    # Receive empty start message and flip on switch
    def start_callback(self, msg):



        if self.start == True:
            return



        self.start = True
     #   navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
        inspection_route = [
        [2.8, 0.1, -(math.pi/2)],
        [2.8, -0.5, -(math.pi)],
        [0.5, -0.5, -(math.pi/2)],
        [0.5, -0.7, 0.0],
        [2.8, -0.7, math.pi/2],
        [2.8, 2.8, math.pi],
        [-0.1, 0.1, math.pi]]
        

        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.1
        initial_pose.pose.position.y = 0.1
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Send our route
        inspection_points = []
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        

        for pt in inspection_route:
            inspection_pose.pose.position.x = pt[0]
            inspection_pose.pose.position.y = pt[1]
            quaternion = calculate_quaternion(pt[2])
            inspection_pose.pose.orientation.x = quaternion[0]
            inspection_pose.pose.orientation.y = quaternion[1]
            inspection_pose.pose.orientation.z = quaternion[2]
            inspection_pose.pose.orientation.w = quaternion[3]
            inspection_points.append(deepcopy(inspection_pose))

        self.navigator.followWaypoints(inspection_points)

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstation
        # i = 0
        # while not navigator.isTaskComplete():
        #     i += 1
        #     feedback = navigator.getFeedback()
        #     if feedback and i % 5 == 0:
        #         print('Executing current waypoint: ' +
        #             str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        # result = navigator.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Inspection of shelves complete! Returning to start...')
        # elif result == TaskResult.CANCELED:
        #     print('Inspection of shelving was canceled. Returning to start...')
        # elif result == TaskResult.FAILED:
        #     print('Inspection of shelving failed! Returning to start...')

        # while not navigator.isTaskComplete():
        #     pass

        # rclpy.shutdown()

    #callback to update our positionlby doing some math
    def populate(self, msg):
        self.position_info = msg.pose.pose


def main():
    
    rclpy.init()

    ser = serial.Serial(
            port='/dev/ttyUSB0', # USB number could change depending on what port the USB is plugged into. DOUBLE CHECK THIS
            baudrate=9600)
    
    team_number = chr(ser.read()[-1])

    test_student = TestStudent(2)
  
    rclpy.spin(test_student)
    
    # Gracefully kill
    test_student.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()