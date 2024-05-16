"""
A Simple Linear Motion Gate


linear: center point
#   2 - - - - - - - 1 
#   |               |
#   |       c       |
#   |               |
#   4 - - - - - - - 3 

"""

import numpy as np
from high_mpc.common.door_index import *
from high_mpc.common.util import Point

class Door_v0(object):
    #
    def __init__(self, center_point, dt):#, rand_k):
        self.s_dim = 2
        self.a_dim = 0
        #self.rand_k = rand_k
        #
        self._kdamping = np.random.uniform(low=0.0005, high=0.0015)
        self._mass = 2.0
        self._gz = 9.81
        self._dt = dt
        #self.pivot_point = pivot_point # e.g., np.array([2.0, 0.0, 2.0])
        self.center_point = center_point
        
        self._state = np.zeros(shape=self.s_dim)
        
        # initial state
        self._motion_range = np.array([-0.5, 0.5])
        self._motion_speed = np.array([0, 0.5])
        self._motion_angle = np.array([np.pi/6,np.pi/3])

        # x, y, z, roll, pitch, yaw, vx, vy, vz
        self.obs_low = np.array([-10, -10, -10, -np.pi, -np.pi, -np.pi, -10, -10, -10])
        self.obs_high = np.array([10, 10, 10, np.pi, np.pi, np.pi, 10, 10, 10])

        self.length = 2.0  # distance between pivot point to the gate center
        self.width = 0.6   # gate width (for visualization only)
        self.height = 0.5  # gate heiht (for visualization only)
            
        #
        self.reset()
        self._t = 0.0

    '''
    def _init_corners(self):  #save or not?
        # compute distance between pivot point to four corners
        # and the 4 angles (for visualization)
        edge1, edge2 = self.width/2,  self.height/2
        self.length1 = np.sqrt( (edge1)**2 + (edge2)**2 )
        self.delta_theta1 = np.arctan2(edge1, edge2)
        #
        self.length2 = self.length1
        self.delta_theta2 = -self.delta_theta1

        #
        edge1, edge2 = self.width/2, self.length+self.height/2
        self.length3 = np.sqrt( (edge1)**2 + (edge2)**2 )
        self.delta_theta3 = np.arctan2(edge1, edge2)
        #
        self.length4 = self.length3
        self.delta_theta4 = -self.delta_theta3
    '''
        
    def reset(self, init_pos=None):
        if init_pos is not None:
            self._state[0] = init_pos
        else:
            self._state[0] = np.random.uniform( \
                low=self._motion_range[0], high=self._motion_range[1])
        
        dir_init = np.random.uniform( \
            low=0, high=1)
        
        if dir_init > 0.5:
            self._state[1] = 0.3+np.random.uniform( \
                low=self._motion_speed[0], high=self._motion_speed[1])
        else:
            self._state[1] = -0.3-np.random.uniform( \
                low=self._motion_speed[0], high=self._motion_speed[1])
        #
        self._t = 0.0
        # print("init pendulum: ", self._state)
        ang_init = np.random.uniform( \
            low=0, high=1)
        if ang_init > 0.5:
            self.angle = np.random.uniform(low=self._motion_angle[0], high=self._motion_angle[1])
        else:
            self.angle = -np.random.uniform(low=self._motion_angle[0], high=self._motion_angle[1])

        return self._state

    def run(self,):
        self._t = self._t + self._dt
        
        # rk4 int
        M = 4
        DT = self._dt/M
        
        X = self._state
        for _ in range(M):
            k1 = DT * self._f(X)
            k2 = DT * self._f(X + 0.5 * k1)
            k3 = DT * self._f(X + 0.5 * k2)
            k4 = DT * self._f(X + k3)
            #
            X = X + (k1 + 2.0*(k2 + k3) + k4)/6.0
        #
        self._state = X
        return self._state

    def _f(self, state):
        #
        current_motion = state[0]
        current_speed = state[1]

        

        return np.array([current_speed, \
            -self._kdamping*current_speed/self._mass])
    #        -self._kdamping*self.rand_k*current_speed/self._mass])


    def get_state(self,):
        return self._state
        
    def get_cartesian_state(self):
        cartesian_state = np.zeros(shape=9)
        cartesian_state[0:3] = self.get_position()
        cartesian_state[3:6] = self.get_euler()
        cartesian_state[6:9] = self.get_veloctiy()
        return cartesian_state
    
    def get_position(self,):
        pos = np.zeros(shape=3)
        pos[0] = self.center_point[0]+self._state[0]*np.sin(self.angle)
        pos[1] = self.center_point[1]+self._state[0]*np.cos(self.angle)
        pos[2] = self.center_point[2]
        return pos

    def get_veloctiy(self,):
        vel = np.zeros(shape=3)
        vel[0] = self._state[1]*np.sin(self.angle)
        vel[1] = self._state[1]*np.cos(self.angle)
        vel[2] = 0.0
        return vel

    def get_euler(self,):
        euler = np.zeros(shape=3)
        euler[0] = 0.0
        euler[1] = 0.0 
        euler[2] = 0.0 
        return euler

    @property
    def t(self):
        return self._t

    @staticmethod
    def _to_planar_coordinates(pivot_point, l, theta):
        y = pivot_point[1] + l*np.sin(theta)
        z = pivot_point[2] - l*np.cos(theta)
        return y, z

    def get_corners(self, ):

        edge1, edge2 = self.width/2,  self.height/2

        y1 = self.center_point[1]+edge1+self._state[0]*np.cos(self.angle)
        z1 = self.center_point[2]+edge2

        y2 = self.center_point[1]-edge1+self._state[0]*np.cos(self.angle)
        z2 = self.center_point[2]+edge2

        y3 = self.center_point[1]+edge1+self._state[0]*np.cos(self.angle)
        z3 = self.center_point[2]-edge2

        y4 = self.center_point[1]-edge1+self._state[0]*np.cos(self.angle)
        z4 = self.center_point[2]-edge2
        
        #
        corners = [ Point(x=y1, y=z1), Point(x=y2, y=z2), Point(x=y3, y=z3), Point(x=y4, y=z4) ]
        return corners

    def get_3d_corners(self,):

        edge1, edge2 = self.width/2,  self.height/2

        y1 = self.center_point[1]+edge1+self._state[0]*np.cos(self.angle)
        z1 = self.center_point[2]+edge2

        y2 = self.center_point[1]-edge1+self._state[0]*np.cos(self.angle)
        z2 = self.center_point[2]+edge2

        y3 = self.center_point[1]+edge1+self._state[0]*np.cos(self.angle)
        z3 = self.center_point[2]-edge2

        y4 = self.center_point[1]-edge1+self._state[0]*np.cos(self.angle)
        z4 = self.center_point[2]-edge2
        #
        x = self.center_point[0]+self._state[0]*np.sin(self.angle)
        corners_3d = [[x, y1, z1], [x, y2, z2 ], [x, y3, z3 ], [x, y4, z4]]
        return corners_3d
