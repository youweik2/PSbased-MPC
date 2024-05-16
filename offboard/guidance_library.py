#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from pyquaternion import Quaternion

from attitude_library import Attitude
from setpoint_buffer import SetpointBuffer

import time
import math
import numpy as np

class FlightModes():

    def __init__(self):
        '''
        ROS Services
        '''

        self.armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vehicle Offboard failed")
            return False

class Controller():

    def __init__(self):

        self.attitude = Attitude()
        self.takeoff_height = 1.5 # m
        self.takeoff_speed = 0.3 # m/s

    def construct_target(self, x, y, z, yaw):
        ''' Function which construct the ROS message to be sent to Mavros with the desired x,y,z position and the target yaw '''

        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 1      #FRAME_LOCAL_NED

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE

        target_raw_pose.yaw = yaw

        return target_raw_pose

    def construct_target_velocity(self, vx, vy, vz, yaw):

        target_raw_vel = PositionTarget()
        target_raw_vel.header.stamp = rospy.Time.now()

        target_raw_vel.coordinate_frame = 1      #FRAME_LOCAL_NED

        target_raw_vel.velocity.x = vx
        target_raw_vel.velocity.y = vy
        target_raw_vel.velocity.z = vz

        target_raw_vel.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE

        target_raw_vel.yaw = yaw

        return target_raw_vel

    def current_yaw(self, pose):
        ''' Takes as input a PoseStamped message and outputs its yaw angle in radiants '''
        quaternion_orientation = pose.pose.orientation
        q1 = quaternion_orientation.x
        q2 = quaternion_orientation.y
        q3 = quaternion_orientation.z
        q4 = quaternion_orientation.w

        quaternion = [q1, q2, q3, q4]
        euler = self.attitude.quatToEuler(quaternion)
        yaw = euler[2]

        #wrap to 2pi
        if yaw > 2*np.pi:
            yaw = yaw - 2*np.pi
        if yaw < 0:
            yaw = yaw + 2*np.pi
        return yaw

    def move_and_yaw(self, x, y, z, yaw_degree, pose):
        ''' Takes as input the desired x,y,z position, the desired yaw in degrees and the current pose in a PoseStamped message and outputs the target pose to be published '''

        yaw_rad = yaw_degree / 180.0 * np.pi
        cur_target_pose = self.construct_target(x, y, z, yaw_rad)
        return cur_target_pose

    def linear_turn(self, psi_in, psi_fin, ti, tf):

        t = time.time()
        if abs(psi_fin - psi_in) < 180:
            psi = psi_in + (psi_fin - psi_in) / (tf - ti) * (t - ti)
        else:
            if psi_fin - psi_in >= 180:
                psi = psi_in + (psi_fin - psi_in - 360) / (tf - ti) * (t - ti)
            if psi_fin - psi_in <= -180:
                psi = psi_in + (psi_fin - psi_in + 360) / (tf - ti) * (t - ti)
        return psi

    def linear_trajectory(self, xi, yi, zi, xf, yf, zf, ti, tf):

        t = time.time()
        if t <= tf:
            x = xi + (xf - xi) / (tf - ti) * (t - ti)
            y = yi + (yf - yi) / (tf - ti) * (t - ti)
            z = zi + (zf - zi) / (tf - ti) * (t - ti)
        else:
            x = xf
            y = yf
            z = zf
        return x, y, z

    def is_at_position(self, x, y, z, pose, toll):
        ''' Takes as input the current pose in a PoseStamped message and desired x, y and z and returns if the robot is in that position (in a defined tolerance range) '''

        x_curr = pose.pose.position.x
        y_curr = pose.pose.position.y
        z_curr = pose.pose.position.z

        if abs(x - x_curr) < toll and abs(y - y_curr) < toll and abs(z - z_curr) < toll:
              return True
        else:
              return False

    def is_at_radius(self, x, y, z, pose, toll):
        ''' Takes as input the current pose in a PoseStamped message and desired x, y and z and returns if the distance is less than a defined threshold '''

        x_curr = pose.pose.position.x
        y_curr = pose.pose.position.y
        z_curr = pose.pose.position.z

        dist_from_target = np.sqrt( (x - x_curr)**2 + (y - y_curr)**2 + (z - z_curr)**2 )

        if dist_from_target < toll:
              return True
        else:
              return False

    def is_at_angle(self, yaw, pose, toll):
        ''' Takes as input the current pose in a PoseStamped message and desired yaw in degrees and returns if the robot is in that orientation (in a defined tolerance range) '''

        current_heading = self.current_yaw(pose)
        if abs(yaw - current_heading / np.pi * 180.0) < toll:
              return True
        else:
              return False

    def is_at_height(self, x, y, z, pose, toll):
        ''' Takes as input the current pose in a PoseStamped message and desired x, y and z and returns if the distance is less than a defined threshold '''

        x_curr = pose.pose.position.x
        y_curr = pose.pose.position.y
        z_curr = pose.pose.position.z

        dist_from_target = abs(z - z_curr)

        if dist_from_target < toll:
              return True
        else:
              return False
