#!/usr/bin/env python3

from pathlib import Path
import rclpy
import numpy as np
from rclpy.node import Node
from crazyflie_py import Crazyswarm
from crazyflie_interfaces.msg import Position
from crazyflie_py.uav_trajectory import Trajectory
from geometry_msgs.msg import PoseStamped

import csv


class PositionNode(Node):

    def __init__(self):
        super().__init__('position_node_publisher')
        self.status_sub = self.create_subscription(
            PoseStamped,
            '/cf_0/pose',
            self.cf_pose_cb,
            10)
        
        self.publisher = self.create_publisher(Position,
                                               '/cf_0/cmd_position',
                                               10)
        
        timer_period = 0.10
        self.tolerence = 0.2
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.i = 0

        with open('/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/circle.csv', 
                  newline='') as traj:  
            reader = csv.reader(traj, delimiter=",")
            next(reader)
            self.traj_points = [list(map(float, row)) for row in reader if row]
            # self.traj_points = csv.reader(traj, delimiter=",")

            #  for row in points:
                 
            #      print(', '.join(row))



    def cf_pose_cb(self, msg):
        self.x_odom = msg.pose.position.x
        self.y_odom = msg.pose.position.y
        self.z_odom = msg.pose.position.z
        # print(f'x = {x:.3f}, y = {y:.3f}, z = {z:.3f}')

    def trajectory(self,msg):

        self.publisher.publish(msg)

        if (abs(msg._x - self.x_odom) > self.tolerence
            and 
            abs(msg._y - self.y_odom) > self.tolerence):
            #             and
            # abs(msg._z - self.z_odom) > self.tolerence

            # print(f'x_odom = {self.x_odom}      pos_x : {msg._x}')
            print(f'visiting point  -->     {msg._x:.2f} {msg._y:.2f} {msg._z:.2f}')
            print(f'actual position -->     {self.x_odom:.2f} {self.y_odom:.2f} {self.z_odom:.2f}')
            print(f'position errors -->     {msg._x - self.x_odom:.2f} {msg._y - self.y_odom:.2f} {msg._z - self.z_odom:.2f}')
            print(f'#######################################')

        else:
            
            print("NEXT POINT")
            self.i = self.i + 1
    


        

    def cmdloop_callback(self):


        pos = Position()
        pos._x = self.traj_points[self.i][1]
        pos._y = self.traj_points[self.i][2]
        pos._z = -self.traj_points[self.i][3]
        pos._yaw = self.traj_points[self.i][4]

        self.trajectory(pos)


def main(args=None):
    rclpy.init(args=args)
    positionNode = PositionNode()

    rclpy.spin(positionNode)

    positionNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
