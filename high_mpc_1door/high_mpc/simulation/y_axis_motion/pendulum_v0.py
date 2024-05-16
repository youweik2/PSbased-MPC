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
from high_mpc.common.pend_index import *
from high_mpc.common.util import Point

class Pendulum_v0(object):
    #
    def __init__(self, center_point, dt):#, rand_k):
        self.s_dim = 2
        self.a_dim = 0
        #self.rand_k = rand_k
        #
        self._kdamping = 0.001
        self._mass = 2.0
        self._gz = 9.81
        self._dt = dt
        #self.pivot_point = pivot_point # e.g., np.array([2.0, 0.0, 2.0])
        self.center_point = center_point
        
        self._state = np.zeros(shape=self.s_dim)
        
        # initial state
        self._motion_range = np.array([-0.5, 0.5])
        self._motion_speed = np.array([0, 0.5])

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
        
    def reset(self, init_ypos=None):
        if init_ypos is not None:
            self._state[0] = init_ypos
        else:
            self._state[0] = np.random.uniform( \
                low=self._motion_range[0], high=self._motion_range[1])
        
        dir_init = np.random.uniform( \
            low=0, high=1)
        
        if dir_init > 0.5:
            self._state[1] = 0.5+np.random.uniform( \
                low=self._motion_speed[0], high=self._motion_speed[1])
        else:
            self._state[1] = -0.5-np.random.uniform( \
                low=self._motion_speed[0], high=self._motion_speed[1])
        #
        self._t = 0.0
        # print("init pendulum: ", self._state)
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
        pos[0] = self.center_point[0]
        pos[1] = self._state[0]
        pos[2] = self.center_point[2]
        return pos

    def get_veloctiy(self,):
        vel = np.zeros(shape=3)
        vel[0] = 0.0
        vel[1] = self._state[1]
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

        y1 = self.center_point[1]+edge1+self._state[0]
        z1 = self.center_point[2]+edge2

        y2 = self.center_point[1]-edge1+self._state[0]
        z2 = self.center_point[2]+edge2

        y3 = self.center_point[1]+edge1+self._state[0]
        z3 = self.center_point[2]-edge2

        y4 = self.center_point[1]-edge1+self._state[0]
        z4 = self.center_point[2]-edge2
        
        #
        corners = [ Point(x=y1, y=z1), Point(x=y2, y=z2), Point(x=y3, y=z3), Point(x=y4, y=z4) ]
        return corners

    def get_3d_corners(self,):

        edge1, edge2 = self.width/2,  self.height/2

        y1 = self.center_point[1]+edge1+self._state[0]
        z1 = self.center_point[2]+edge2

        y2 = self.center_point[1]-edge1+self._state[0]
        z2 = self.center_point[2]+edge2

        y3 = self.center_point[1]+edge1+self._state[0]
        z3 = self.center_point[2]-edge2

        y4 = self.center_point[1]-edge1+self._state[0]
        z4 = self.center_point[2]-edge2
        #
        x = self.center_point[0]
        corners_3d = [[x, y1, z1], [x, y2, z2 ], [x, y3, z3 ], [x, y4, z4]]
        return corners_3d





class Pendulum_v1(object):
    #
    def __init__(self, center_point, sigma, T, dt):#, rand_k):
        self.s_dim = 2
        self.a_dim = 0
        self._length = 2.0   
        self._kdamping = 0.001
        self._mass = 2.0
        self._pi = 3.141592
        self._gz = 9.81
        self._dt = dt
        self.center_point = center_point # e.g., np.array([2.0, 0.0, 2.0])
        self._T = T
        #
        self.sigma = sigma
        self._N = int(T/dt)
        self.width = 2.0
        self.height = 1
        #self.rand_k = rand_k
    
    def plan(self, state, opt_t=1.0):
        #
        plans, pred_traj = [], []
        M = 4
        DT = self._dt/M
        #
        for i in range(self._N):
            X = state
            for _ in range(M):
                k1 = DT * self._f(X)
                k2 = DT * self._f(X + 0.5 * k1)
                k3 = DT * self._f(X + 0.5 * k2)
                k4 = DT * self._f(X + k3)
                #
                X = X + (k1 + 2.0*(k2 + k3) + k4)/6.0
            #
            state = X
            traj_euler_point = self.get_cartesian_state(state).tolist()
            
            # plan trajectory and optimal time & optimal vx
            traj_quat_point = self.get_cartesian_state(state).tolist()
            # traj_quat_point[kPosX] = opt_vx
            
            current_t = i * self._dt
            plan_i = traj_quat_point + [current_t, opt_t, self.sigma]
    
            #
            plans += plan_i
            pred_traj.append(traj_euler_point)
        
        return plans, pred_traj

    def _f(self, state):
        #
        current_motion = state[0]
        current_speed = state[1]

        return np.array([current_speed, \
            -self._kdamping*current_speed/self._mass])
        #    -self._kdamping*self.rand_k*current_speed/self._mass])


    def get_state(self,):
        return self._state
        
    def get_cartesian_state(self,state):
        cartesian_state = np.zeros(shape=9)
        cartesian_state[0:3] = self.get_position(state)
        cartesian_state[3:6] = self.get_euler(state)
        cartesian_state[6:9] = self.get_veloctiy(state)
        return cartesian_state
    
    def get_position(self,state):
        pos = np.zeros(shape=3)
        pos[0] = self.center_point[0]
        pos[1] = state[0]
        pos[2] = self.center_point[1]
        return pos

    def get_veloctiy(self,state):
        vel = np.zeros(shape=3)
        vel[0] = 0.0
        vel[1] = state[1]
        vel[2] = 0.0
        return vel

    def get_euler(self,state):
        euler = np.zeros(shape=3)
        euler[0] = 0.0
        euler[1] = 0.0 
        euler[2] = 0.0 
        return euler
    
    def get_quaternion(self, state):
        roll, pitch, yaw = self.get_euler(state)
        #
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        #
        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr
        #
        return [qw, qx, qy, qz]



if __name__ == "__main__":
    # test run
    import matplotlib.pyplot as plt
    dt = 0.02
    tf = 20.0
    #
    pivot = [2.0, 2.0, 2.0] # x, y, z

    # # # # # # # # # # # # # # # # # # #
    # -- test Pendulum v0
    # # # # # # # # # # # # # # # # # #
    env = Pendulum_v0(pivot, dt=0.02)
    l_t, l_pos, l_vel, l_theta  = [], [], [], []
    #
    env.reset()
    #
    while env.t < tf:
        #
        l_t.append(env.t)
        l_pos.append(env.get_position())
        l_vel.append(env.get_veloctiy())
        l_theta.append(env.get_euler())
        #
        env.run()
    #
    l_pos = np.asarray(l_pos)
    l_vel = np.asarray(l_vel)
    l_theta = np.asarray(l_theta)
    #
    fig, axes = plt.subplots(3, 1) 
    axes[0].plot(l_t, l_pos[:, 0], '-r', label="x")
    axes[0].plot(l_t, l_pos[:, 1], '-g', label="y")
    axes[0].plot(l_t, l_pos[:, 2], '-b', label="z")
    axes[0].legend()
    #
    axes[1].plot(l_t, l_vel[:, 0], '-r', label="vx")
    axes[1].plot(l_t, l_vel[:, 1], '-g', label="vy")
    axes[1].plot(l_t, l_vel[:, 2], '-b', label="vz")
    axes[1].legend()
    #
    axes[2].plot(l_t, l_theta[:, 0], '-r', label="roll")
    axes[2].plot(l_t, l_theta[:, 1], '-g', label="pitch")
    axes[2].plot(l_t, l_theta[:, 2], '-b', label="yaw")
    axes[2].legend()

    #
    plt.show()