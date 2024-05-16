#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion

from attitude_library import Attitude
from guidance_library import *

import time
import math
import numpy as np

class PathGenerationNode:

    def pose_callback(self, msg):
        self.local_pose = msg

    def __init__(self):

        rospy.init_node("commander_node")

        '''Initialization'''
        self.local_pose = PoseStamped()
        self.attitude = Attitude()
        self.controller = Controller()
        self.mode = FlightModes()

        rate = 20
        self.rate = rospy.Rate(rate)
        self.disarm_state = False

        '''Subscribe to local position'''
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_callback)

        '''Publish the pose target to the commander topic'''
        self.pose_target_pub = rospy.Publisher('commander/set_pose', PositionTarget, queue_size=10)

        ''' Desired waypoints and speed '''
        self.waypoint_list_x = [0, -2, -2, 1, 0] # position setpoint along local East, m
        self.waypoint_list_y = [1, 1, -1, -1, 0] # position setpoint along local North, m
        self.height = self.controller.takeoff_height
        self.yaw_sp = [90, 180, 270, 360, 135] # yaw angle setpoint (positive CCW from above and zero aligned to East, consistent with ENU notation), deg
        self.speed = 0.1 # cruise speed, m/s
        self.landing_speed = 0.7 # m/s
        self.speed_yaw = 15.0 # max yaw rate, deg/s

        if len(self.waypoint_list_x) != len(self.waypoint_list_y):
            print("Error. The lists of waypoint in x and y are different")

        self.wp_number = len(self.waypoint_list_x)
        print("Mission received. Number of waypoint: {}".format(self.wp_number))

        time.sleep(5)

    def wrap_to_2pi(self, angle):

        if angle > 2 * np.pi:
            angle = angle - 2 * np.pi

        if angle < 0:
            angle = angle + 2 * np.pi

        return angle

    def rad2deg(self, angle_rad):

        angle_deg = angle_rad / np.pi * 180.0

        return angle_deg

    def deg2rad(self, angle_deg):

        angle_rad = angle_deg / 180.0 *  np.pi

        return angle_rad

    def land(self):

        yaw_wp = self.rad2deg( self.controller.current_yaw(self.local_pose) )
        yaw_rad = self.deg2rad(yaw_wp)

        vx = 0
        vy = 0
        vz = -self.landing_speed

        zf = self.controller.takeoff_height

        ti = time.time()
        tf = ti + zf / self.landing_speed
        tf_increased = tf + 1.0

        target_vel = self.controller.construct_target_velocity(vx, vy, vz, yaw_rad)

        while time.time() < tf_increased:
            self.pose_target_pub.publish(target_vel)
            self.rate.sleep()

    def rotate_yaw(self, yaw_wp, z_wp):

        print("Rotate yaw: {0:.2f} deg".format(yaw_wp))

        self.x_in = self.local_pose.pose.position.x
        self.y_in = self.local_pose.pose.position.y
        self.yaw_in = self.rad2deg( self.controller.current_yaw(self.local_pose) )                         #already wrapped to 2pi

        yaw_wp_input = yaw_wp
        yaw_wp = self.rad2deg(self.wrap_to_2pi(self.deg2rad(yaw_wp_input)))

        # compute angle in degrees to turn
        delta_yaw = yaw_wp - self.yaw_in

        if abs(delta_yaw) > 180:
            abs_delta_yaw = 360 - abs(delta_yaw)
        else:
            abs_delta_yaw = abs(delta_yaw)

        # compute time
        dt = abs_delta_yaw / self.speed_yaw
        ti = time.time()
        tf = ti + dt

        while time.time() < tf:
            psi = self.controller.linear_turn(self.yaw_in, yaw_wp, ti, tf)
            target_controller = self.controller.move_and_yaw(self.x_in, self.y_in, z_wp, psi, self.local_pose)
            self.pose_target_pub.publish(target_controller)
            self.rate.sleep()

    def go_to_wp(self, x_wp, y_wp, z_wp):

        print("Go to waypoint: (x = {0:.2f}, y = {1:.2f} )".format(x_wp, y_wp))

        self.x_in = self.local_pose.pose.position.x
        self.y_in = self.local_pose.pose.position.y
        self.z_in = self.local_pose.pose.position.z

        distance = np.sqrt( (x_wp - self.x_in)**2 + (y_wp - self.y_in)**2 + (z_wp - self.z_in)**2 )
        occurred_time = distance / self.speed
        ti = time.time()
        tf = ti + occurred_time
        increased_tf = tf + 1.0

        yaw_wp = self.rad2deg( self.controller.current_yaw(self.local_pose) )

        while time.time() < increased_tf:
            x, y, z = self.controller.linear_trajectory(self.x_in, self.y_in, self.z_in, x_wp, y_wp, z_wp, ti, tf)
            target_controller = self.controller.move_and_yaw(x, y, z, yaw_wp, self.local_pose)
            self.pose_target_pub.publish(target_controller)
            self.rate.sleep()

    def simple_path(self):

        for i in range(0, self.wp_number):

            self.x_in = self.local_pose.pose.position.x
            self.y_in = self.local_pose.pose.position.y
            self.z_in = self.local_pose.pose.position.z
            print("Initial position: x={0:.2f}, y={1:.2f}, z={2:.2f} m".format(self.x_in, self.y_in, self.z_in))

            self.x_sp_i = self.waypoint_list_x[i]
            self.y_sp_i = self.waypoint_list_y[i]
            self.yaw_sp_i = self.yaw_sp[i]

            self.rotate_yaw(self.yaw_sp_i, self.height)
            self.go_to_wp(self.x_sp_i, self.y_sp_i, self.height)

            print("Reached waypoint")
            time.sleep(2)

        print("Finished waypoints. Landing...")
        self.land()

        self.disarm_state = False
        print("The drone has landed")
        while not self.disarm_state:
            self.disarm_state = self.mode.disarm()
            print("Disarming")

if __name__ == '__main__':

    con = PathGenerationNode()
    con.simple_path()
