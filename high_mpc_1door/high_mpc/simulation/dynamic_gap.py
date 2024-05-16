import numpy as np
#
from high_mpc.simulation.quadrotor import Quadrotor_v0
from high_mpc.simulation.door_v0 import Door_v0
from high_mpc.simulation.door_v1 import Door_v1
#
from high_mpc.common.quad_index import *

#
class Space(object):

    def __init__(self, low, high):
        self.low = low
        self.high = high
        self.shape = self.low.shape

    def sample(self):
        return np.random.uniform(self.low, self.high)

class DynamicGap(object):

    def __init__(self, mpc, plan_T, plan_dt):
        #
        self.krandom = 0
        self.mpc = mpc
        self.plan_T = plan_T
        self.plan_dt = plan_dt

        # 
        self.goal_point = np.array([4.5,  0.0, 2.0])
        self.start_point = np.array([2.0, 0.0, 1.0])

        # goal state, position, quaternion, velocity
        self.quad_sT = self.goal_point.tolist() + [1.0, 0.0, 0.0, 0.0] + [0.0, 0.0, 0.0] 

        # simulation parameters ....
        self.sim_T = 3.0    # Episode length, seconds
        self.sim_dt = 0.02  # simulation time step
        self.max_episode_steps = int(self.sim_T/self.sim_dt)
        # Simulators, a quadrotor and a moving door
        self.quad = Quadrotor_v0(dt=self.sim_dt)
        self.door = Door_v0(self.start_point, dt=self.sim_dt)

        self.planner = Door_v1(center_point=self.start_point, sigma=10, \
            T=self.plan_T, dt=self.plan_dt)
    

        #
        self.observation_space = Space(
            low=np.array([-10.0, -10.0, -10.0, -2*np.pi, -2*np.pi, -2*np.pi, -10.0, -10.0, -10.0]),
            high=np.array([10.0, 10.0, 10.0, 2*np.pi, 2*np.pi, 2*np.pi, 10.0, 10.0, 10.0]),
        )

        self.action_space = Space(
            low=np.array([0.0]),
            high=np.array([2*self.plan_T])
        )

        # reset the environment
        self.t = 0
        self.reset()
    
    def seed(self, seed):
        np.random.seed(seed=seed)
    
    def reset(self, init_ypos=None):
        self.t = 0
        # state for ODE
        #self.quad_state = np.array([0,0,1,1,0,0,0,0,0,0]) 
        self.quad_state = self.quad.reset()
        if init_ypos is not None:
            self.door_state = self.door.reset(init_ypos)
        else:
            self.door_state = self.door.reset()
        
        # observation, can be part of the state, e.g., postion
        # or a cartesian representation of the state
        #quad_obs = np.array([0,0,1,0,0,0,0,0,0]) 
        quad_obs = self.quad.get_cartesian_state()
        door_obs = self.door.get_cartesian_state()
        #
        obs = (quad_obs - door_obs).tolist()
        
        return obs

    def step(self, u=0):
        #print(self.quad_state)
        self.t += self.sim_dt
        opt_t = u
        
        #
        plan_door_traj, pred_door_traj_cart = self.planner.plan(self.door_state, opt_t)
        pred_door_traj_cart = np.array(pred_door_traj_cart)
        
        #
        quad_s0 = self.quad_state.tolist()
        ref_traj = quad_s0 + plan_door_traj + self.quad_sT

        # run nonliear model predictive control
        quad_act, pred_traj = self.mpc.solve(ref_traj)

        # run the actual control command on the quadrotor
        self.quad_state = self.quad.run(quad_act)
        #print("act",quad_act)class DynamicGap(object):

    def __init__(self, mpc, plan_T, plan_dt):
        #
        self.krandom = 0
        self.mpc = mpc
        self.plan_T = plan_T
        self.plan_dt = plan_dt

        # 
        self.goal_point = np.array([4.5,  0.0, 2.0])
        self.start_point = np.array([2.0, 0.0, 1.0])

        # goal state, position, quaternion, velocity
        self.quad_sT = self.goal_point.tolist() + [1.0, 0.0, 0.0, 0.0] + [0.0, 0.0, 0.0] 

        # simulation parameters ....
        self.sim_T = 4.0    # Episode length, seconds
        self.sim_dt = 0.02  # simulation time step
        self.max_episode_steps = int(self.sim_T/self.sim_dt)
        # Simulators, a quadrotor and a moving door
        self.quad = Quadrotor_v0(dt=self.sim_dt)
        self.door = Door_v0(self.start_point, dt=self.sim_dt)

        self.planner = Door_v1(center_point=self.start_point, sigma=10, \
            T=self.plan_T, dt=self.plan_dt)
    

        #
        self.observation_space = Space(
            low=np.array([-10.0, -10.0, -10.0, -2*np.pi, -2*np.pi, -2*np.pi, -10.0, -10.0, -10.0]),
            high=np.array([10.0, 10.0, 10.0, 2*np.pi, 2*np.pi, 2*np.pi, 10.0, 10.0, 10.0]),
        )

        self.action_space = Space(
            low=np.array([0.0]),
            high=np.array([2*self.plan_T])
        )

        # reset the environment
        self.t = 0
        self.reset()
    
    def seed(self, seed):
        np.random.seed(seed=seed)
    
    def reset(self, init_ypos=None):
        self.t = 0
        # state for ODE
        #self.quad_state = np.array([0,0,1,1,0,0,0,0,0,0]) 
        self.quad_state = self.quad.reset()
        if init_ypos is not None:
            self.door_state = self.door.reset(init_ypos)
        else:
            self.door_state = self.door.reset()
        
        # observation, can be part of the state, e.g., postion
        # or a cartesian representation of the state
        #quad_obs = np.array([0,0,1,0,0,0,0,0,0]) 
        quad_obs = self.quad.get_cartesian_state()
        door_obs = self.door.get_cartesian_state()
        #
        obs = (quad_obs - door_obs).tolist()
        
        return obs

    def step(self, u=0):
        #print(self.quad_state)
        self.t += self.sim_dt
        opt_t = u
        
        #
        plan_door_traj, pred_door_traj_cart = self.planner.plan(self.door_state, opt_t)
        pred_door_traj_cart = np.array(pred_door_traj_cart)
        
        #
        quad_s0 = self.quad_state.tolist()
        ref_traj = quad_s0 + plan_door_traj + self.quad_sT

        # run nonliear model predictive control
        quad_act, pred_traj = self.mpc.solve(ref_traj)

        # run the actual control command on the quadrotor
        self.quad_state = self.quad.run(quad_act)
        #print("act",quad_act)
        #print(opt_t)
        # simulate one step door
        self.door_state = self.door.run()
        
        # update the observation.
        quad_obs = self.quad.get_cartesian_state()
        door_obs = self.door.get_cartesian_state()
        obs = (quad_obs - door_obs).tolist()
        #print("obs",obs)
        
        info = {
            "quad_obs": quad_obs, 
            "quad_act": quad_act, 
            "quad_axes": self.quad.get_axes(),
            "door_obs": door_obs,
            "door_corners": self.door.get_3d_corners(),
            "pred_quad_traj": pred_traj, 
            "pred_door_traj": pred_door_traj_cart, 
            "opt_t": opt_t, "plan_dt": self.plan_dt}
        done = False

        if self.t >= (self.sim_T-self.sim_dt):
            done = True

        return obs, 0, done, info
    
    def episode(self, u):
        opt_t = u
        #
        plan_door_traj, pred_door_traj_cart = self.planner.plan(self.door_state, opt_t)
        pred_door_traj_cart = np.array(pred_door_traj_cart)
        
        #
        quad_s0 = self.quad_state.tolist()
        ref_traj = quad_s0 + plan_door_traj + self.quad_sT
    
        _, pred_traj = self.mpc.solve(ref_traj)
        
        opt_node = np.clip( int(opt_t/self.plan_dt), 0, pred_traj.shape[0]-1)
        # if quad_s0[kPosX] >= self.pivot_point[kPosX]+0.5:
        #     # obs = self.reset()
        #     loss = np.linalg.norm(pred_traj[opt_node, kPosX:kPosZ+1] - np.tile(self.goal_point, (pred_traj.shape[0], 1)))
        #     rew = - np.mean(loss)             
        # else:    
        opt_min = np.clip(opt_node-10, 0, pred_traj.shape[0]-1)
        opt_max = np.clip(opt_node+5, 0, pred_traj.shape[0]-1)
        # opt_min = np.clip(opt_node-1, 0, pred_traj.shape[0]-1)
        # opt_max = np.clip(opt_node+1, 0, pred_traj.shape[0]-1)
        #
        loss = np.linalg.norm(pred_traj[opt_min:opt_max, kPosX:kPosZ+1]  - pred_door_traj_cart[opt_min:opt_max, kPosX:kPosZ+1])
        rew = - loss
        #    
        return rew, opt_node
    
    @staticmethod
    def _is_within_gap(gap_corners, point):
        A, B, C = [], [], []    
        for i in range(len(gap_corners)):
            p1 = gap_corners[i]
            p2 = gap_corners[(i + 1) % len(gap_corners)]
            
            # calculate A, B and C
            a = -(p2.y - p1.y)
            b = p2.x - p1.x
            c = -(a * p1.x + b * p1.y)

            A.append(a)
            B.append(b)
            C.append(c)
        D = []
        for i in range(len(A)):
            d = A[i] * point.x + B[i] * point.y + C[i]
            D.append(d)

        t1 = all(d >= 0 for d in D)
        t2 = all(d <= 0 for d in D)
        return t1 or t2

    def close(self,):
        return True

    def render(self,):
        return False
        #print(opt_t)
        # simulate one step door
        self.door_state = self.door.run()
        
        # update the observation.
        quad_obs = self.quad.get_cartesian_state()
        door_obs = self.door.get_cartesian_state()
        obs = (quad_obs - door_obs).tolist()
        #print("obs",obs)
        #
        info = {
            "quad_obs": quad_obs, 
            "quad_act": quad_act, 
            "quad_axes": self.quad.get_axes(),
            "door_obs": door_obs,
            "door_corners": self.door.get_3d_corners(),
            "pred_quad_traj": pred_traj, 
            "pred_door_traj": pred_door_traj_cart, 
            "opt_t": opt_t, "plan_dt": self.plan_dt}
        done = False

        if self.t >= (self.sim_T-self.sim_dt):
            done = True

        return obs, 0, done, info
    
    def episode(self, u):
        opt_t = u
        #
        plan_door_traj, pred_door_traj_cart = self.planner.plan(self.door_state, opt_t)
        pred_door_traj_cart = np.array(pred_door_traj_cart)
        
        #
        quad_s0 = self.quad_state.tolist()
        ref_traj = quad_s0 + plan_door_traj + self.quad_sT
    
        _, pred_traj = self.mpc.solve(ref_traj)
        
        opt_node = np.clip( int(opt_t/self.plan_dt), 0, pred_traj.shape[0]-1)
        # if quad_s0[kPosX] >= self.pivot_point[kPosX]+0.5:
        #     # obs = self.reset()
        #     loss = np.linalg.norm(pred_traj[opt_node, kPosX:kPosZ+1] - np.tile(self.goal_point, (pred_traj.shape[0], 1)))
        #     rew = - np.mean(loss)             
        # else:    
        opt_min = np.clip(opt_node-10, 0, pred_traj.shape[0]-1)
        opt_max = np.clip(opt_node+5, 0, pred_traj.shape[0]-1)
        # opt_min = np.clip(opt_node-1, 0, pred_traj.shape[0]-1)
        # opt_max = np.clip(opt_node+1, 0, pred_traj.shape[0]-1)
        #
        loss = np.linalg.norm(pred_traj[opt_min:opt_max, kPosX:kPosZ+1]  - pred_door_traj_cart[opt_min:opt_max, kPosX:kPosZ+1])
        rew = - loss
        #    
        return rew, opt_node
    
    @staticmethod
    def _is_within_gap(gap_corners, point):
        A, B, C = [], [], []    
        for i in range(len(gap_corners)):
            p1 = gap_corners[i]
            p2 = gap_corners[(i + 1) % len(gap_corners)]
            
            # calculate A, B and C
            a = -(p2.y - p1.y)
            b = p2.x - p1.x
            c = -(a * p1.x + b * p1.y)

            A.append(a)
            B.append(b)
            C.append(c)
        D = []
        for i in range(len(A)):
            d = A[i] * point.x + B[i] * point.y + C[i]
            D.append(d)

        t1 = all(d >= 0 for d in D)
        t2 = all(d <= 0 for d in D)
        return t1 or t2

    def close(self,):
        return True

    def render(self,):
        return False
    