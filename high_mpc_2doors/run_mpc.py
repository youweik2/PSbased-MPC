import os
import time
import datetime
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#
from functools import partial

#
from high_mpc.simulation.dynamic_gap import DynamicGap
from high_mpc.mpc.mpc import MPC
from high_mpc.simulation.animation import SimVisual
#
def arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_video', type=bool, default=True,
        help="Save the animation as a video file")
    return parser

def judge(y, z):

    quad_width = 0.5
    quad_height = 0.20

    door_width = 0.8
    door_height = 0.6

    y_sum = np.abs(y)+quad_width/2
    z_sum = np.abs(z)+quad_height/2

    if y_sum > door_width/2 or z_sum > door_height/2:
        crash = 1
        print(y_sum,z_sum)
    else:
        crash = 0

    return crash

def run_mpc(env):
    #
    env.reset()
    t, n = 0, 0
    t0 = time.time()
    flag = 0
    error_detect = []
    aim_t = 0
    crash_times = 0
    error_goal = []

    for i in range(100):
        obs = env.reset()
        t = 0
        n = 0
        crash = 0

        while t < env.sim_T:
            t = env.sim_dt * n
            next_obs, _, _, info = env.step()
            t_now = time.time()
            #print(t_now - t0)
            #
            t0 = time.time()
            #
            n += 1

            if obs[0]<0 and next_obs[0]>0:
                y=obs[0]/(-obs[0]+next_obs[0])*obs[1]+next_obs[0]/(-obs[0]+next_obs[0])*next_obs[1]
                z=obs[0]/(-obs[0]+next_obs[0])*obs[2]+next_obs[0]/(-obs[0]+next_obs[0])*next_obs[2]
                error = [np.sqrt(y**2+z**2)]
                error_detect.append(error)
                aim_t = aim_t + t

            quad_length = 0.37

            if -quad_length/2 < obs[0] < quad_length/2 and crash == 0:
                crash = judge(obs[1],obs[2])
                crash_times = crash_times + crash

            obs = next_obs

            update = False
            if t >= env.sim_T:
                update = True
                final_pos = info["quad_obs"]
                x_f = final_pos[0] - 4.5
                y_f = final_pos[1]
                z_f = final_pos[2] - 2.0
                error = [np.sqrt(x_f**2+y_f**2+z_f**2)]
                error_goal.append(error)

            yield [info, t, update]

            
    aim_t = aim_t/100
    print("through_t",aim_t)
    print("crash_happend_times:",crash_times)
    np.savetxt('error_mpc_y.txt',error_detect)
    np.savetxt('error_fmpc_y.txt',error_goal)



def main():
    #
    args = arg_parser().parse_args()
    #
    plan_T = 2.0   # Prediction horizon for MPC and local planner
    plan_dt = 0.1 # Sampling time step for MPC and local planner
    so_path = "./high_mpc/mpc/saved/mpc_v1.so" # saved mpc model (casadi code generation)
    #
    mpc = MPC(T=plan_T, dt=plan_dt, so_path=so_path)
    env = DynamicGap(mpc, plan_T, plan_dt)
    
    #
    sim_visual = SimVisual(env)

    
    run_mpc(env)
    
    run_frame = partial(run_mpc, env)
    ani = animation.FuncAnimation(sim_visual.fig, sim_visual.update, frames=run_frame,
            init_func=sim_visual.init_animate, interval=100, blit=True, repeat=False)
    
   
    if args.save_video:
        writer = animation.writers["ffmpeg"]
        writer = writer(fps=10, metadata=dict(artist='Me'), bitrate=1800)
        ani.save("MPC_0.mp4", writer=writer)
    
    plt.tight_layout()
    plt.show()
    
if __name__ == "__main__":
    main()
