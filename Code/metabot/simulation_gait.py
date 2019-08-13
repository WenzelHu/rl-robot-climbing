import math
import time
import os
import numpy as np
from Vrep.vrep_simulation import obstacle, environment
from itertools import cycle, chain
from q.rl_q2 import rl

from matplotlib import pyplot as plt
import _pickle as cPickle
###########################################PREDEFINED VARIABLES#########################################################################################

ACTION_SPACE_LENGTH = 729
STATE_SPACE_LENGTH = 891
NUM_EPISOIDE = 100
READ_WEIGHTS = False


##########################################################################################################################################
    

def main():
            
    env = environment()
    env.start_simulation()
    env.set_goal(-1, -1)
    env.update()
    
    RL_solver = rl(ACTION_SPACE_LENGTH, STATE_SPACE_LENGTH)
    RL_solver.set_temperature(1.0)
    weight_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')
    if not os.path.exists(weight_path):
        print('[ERROR] Weights path %s dose not exist, save weights to current path' % weight_path)
        weight_path = os.getcwd()
    
    if READ_WEIGHTS:
        RL_solver.load_q(weight_path)

    reward_hist = []
    
    for i_episoide in range(NUM_EPISOIDE):
        print('Start episoide %d' %(i_episoide))
        
        env.robot_go_initial()
        env.update()
        RL_solver.rl_init(env.states)
        
        for t in range(1000):
            env.action = RL_solver.action_prev
            #env.action = 0
            env.take_action()
            env.update()
            reward = env.get_reward()
            reward_hist.append(reward)
            '''
            if RL_solver.state_prev >= 880:
                RL_solver.state_aft = env.states
                RL_solver.make_policy_aft()
                RL_solver.state_prev = RL_solver.state_aft
                RL_solver.action_prev = RL_solver.action_aft_policy
            else:
            '''
            RL_solver.q_learning(env.states, reward)
            
            isSuccess = (env.distance < 0.05)
            if env.is_incline:
                print('incline too much, episoide stop')
                break
            if isSuccess:
                print('goal reached, episoide stop')
                break
                
        env.stop_simulation()
        print('episoide %d execution finished' %(i_episoide))
        

        RL_solver.save_q(weight_path)
        time.sleep(1)
        env.start_simulation()
        
    env.stop_simulation()
    env.stop_client()
    RL_solver.save_q(weight_path)

	plt.plot(reward_hist)
    with open("./hist_reward.pkl", 'wb') as writefile:
        cPickle.dump(reward_hist, writefile)
	plt.show()

    print ('[INFO] Program ended')

if __name__ == "__main__":
    # execute only if run as a script
    main()
