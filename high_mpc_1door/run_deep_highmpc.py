import os
import datetime
import argparse
import numpy as np
from functools import partial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# A Gym style environment
from high_mpc.simulation.dynamic_gap import DynamicGap
from high_mpc.mpc.high_mpc import High_MPC
from high_mpc.simulation.animation import SimVisual
from high_mpc.common import logger
from high_mpc.common import util as U
from high_mpc.policy import deep_high_policy

#
def arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--option', type=int, default=0,
        help="0 - Data collection; 1 - train the deep high-level policy; 2 - test the trained policy.")
    parser.add_argument('--save_dir', type=str, default=os.path.dirname(os.path.realpath(__file__)),
        help="Directory where to save the checkpoints and training metrics")
    parser.add_argument('--save_video', type=bool, default=True,
        help="Save the animation as a video file")
    parser.add_argument('--load_dir', type=str, help="Directory where to load weights")
    return parser

def judge(y, z):

    quad_width = 0.5
    quad_height = 0.20
    quad_length = 0.37

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

    
def run_deep_high_mpc(env, actor_params, load_dir):
    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]
    #
    actor = deep_high_policy.Actor(obs_dim, act_dim)
    actor.load_weights(load_dir)
    #
    ep_len, ep_reward =  0, 0
    error_detect = []
    error_goal = []
    crash_times = 0
    aim_t = 0

    for i in range(100):
        crash = 0
        obs = env.reset()
        t = 0
        while t < env.sim_T:
            t += env.sim_dt
            #
            obs_tmp = np.reshape(obs, (1, -1)) # to please tensorflow
            act = actor(obs_tmp).numpy()[0]

            # execute action
            next_obs, reward, _, info = env.step(act)


            if obs[0]<0 and next_obs[0]>0:
                y=obs[0]/(-obs[0]+next_obs[0])*obs[1]+next_obs[0]/(-obs[0]+next_obs[0])*next_obs[1]
                z=obs[0]/(-obs[0]+next_obs[0])*obs[2]+next_obs[0]/(-obs[0]+next_obs[0])*next_obs[2]
                error = [np.sqrt(y**2+z**2)]
                aim_t = aim_t + t
                error_detect.append(error)
            
            quad_length = 0.37

            if -quad_length/2 < obs[0] < quad_length/2 and crash == 0:
                crash = judge(obs[1],obs[2])
                crash_times = crash_times + crash

            #
            obs = next_obs
            ep_reward += reward

            

            #
            ep_len += 1

            #
            update = False



            if t >= env.sim_T:
                update = True
                final_pos = info["quad_obs"]
                x_f = final_pos[0] - 4.5
                y_f = final_pos[1]
                z_f = final_pos[2] - 2.0
                #print(x_f,y_f,z_f)
                error = [np.sqrt(x_f**2+y_f**2+z_f**2)]
                error_goal.append(error)



            yield [info, t, update]
        

        
    np.savetxt('error_high_y.txt',error_detect)
    np.savetxt('error_final_y.txt',error_goal)
    aim_t = aim_t/100
    print("through_t",aim_t)
    print("crash_happend_times:",crash_times)

def main():
    #
    args = arg_parser().parse_args()
    #
    plan_T = 2  # Prediction horizon for MPC and local planner 2
    plan_dt = 0.04 # Sampling time step for MPC and local planner 0.04
    so_path = "./high_mpc/mpc/saved/high_mpc.so" # saved high mpc model (casadi code generation)
    #
    high_mpc = High_MPC(T=plan_T, dt=plan_dt, so_path=so_path)
    env = DynamicGap(high_mpc, plan_T, plan_dt)

    #
    actor_params = dict(
        hidden_units=[32, 32],
        learning_rate=1e-4,
        activation='relu',
        train_epoch=1001,
        batch_size=128
    )

    # training
    training_params = dict(
        max_samples =5000,
        max_wml_iter=15,
        beta0=10.0,
        n_samples=15,
    )
    
    # if in training mode, create new dir to save the model and checkpoints
    if args.option == 0: # collect data
        save_dir = U.get_dir(args.save_dir + "/Dataset")
        save_dir = os.path.join(save_dir, datetime.datetime.now().strftime("deep_highmpc-%m-%d-%H-%M-%S"))

        #
        logger.configure(dir=save_dir)
        logger.log("***********************Log & Store Hyper-parameters***********************")
        logger.log("actor params")
        logger.log(actor_params)
        logger.log("training params")
        logger.log(training_params)
        logger.log("***************************************************************************")

        # 
        deep_high_policy.data_collection(env=env, logger=logger, \
            save_dir=save_dir, **training_params)
    elif args.option == 1: # train the policy
        data_dir = args.save_dir + "/Dataset"
        deep_high_policy.train(env, logger, data_dir, **actor_params)
    elif args.option == 2: # evaluate the policy
        load_dir = args.save_dir + "/Dataset/act_net/weights_900.h5"
        sim_visual = SimVisual(env)
        run_frame = partial(run_deep_high_mpc, env, actor_params, load_dir)
        ani = animation.FuncAnimation(sim_visual.fig, sim_visual.update, frames=run_frame, 
            init_func=sim_visual.init_animate, interval=100, blit=True, repeat=False)
        # #
        if args.save_video:
            writer = animation.writers["ffmpeg"]
            writer = writer(fps=10, metadata=dict(artist='Me'), bitrate=1800)
            ani.save("output.mp4", writer=writer)
        # #
        plt.tight_layout()
        # plt.axis('off')
        plt.show()
        
if __name__ == "__main__":
    main()