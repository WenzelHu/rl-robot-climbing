import math
import time
import os
import numpy as np
from Vrep.vrep_simulation import obstacle, environment
from itertools import cycle, chain
from q.rl_q2 import rl
from matplotlib import pyplot as plt
###########################################PREDEFINED VARIABLES#########################################################################################

ACTION_SPACE_LENGTH = 729
STATE_SPACE_LENGTH = 891
NUM_EPISOIDE = 100
READ_WEIGHTS = False


##########################################################################################################################################
    

def main():
    q_value = list()
            
    env = environment()
    env.start_simulation()
    env.set_goal(-2, -2)
    env.update()
    
    RL_solver = rl(ACTION_SPACE_LENGTH, STATE_SPACE_LENGTH)
    RL_solver.set_temperature(1.0)
    weight_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')
    if not os.path.exists(weight_path):
        print('[ERROR] Weights path %s dose not exist, save weights to current path' % weight_path)
        weight_path = os.getcwd()
    
    if READ_WEIGHTS:
        RL_solver.load_q(weight_path)
        
    #for action in range(ACTION_SPACE_LENGTH):
    for action in range(1):
        for i_episoide in range(NUM_EPISOIDE):
            print('Start episoide %d for action %d' %(i_episoide, action))
            env.robot_go_initial()
            env.update()
            RL_solver.rl_init(env.states)
            pre_qvalue = RL_solver.q[env.states, action]
            print("pre_qvalue:", pre_qvalue)
            finish_flag = False
        
            for t in range(100):
                env.action = action
                env.take_action()
                env.update()
                reward = env.get_reward()
                RL_solver.action_prev = action
                RL_solver.action_aft_policy = action
                RL_solver.q_learning(env.states, reward)
                curr_qvalue = RL_solver.q[env.states, action]
                q_value.append(curr_qvalue)
                print("curr_qvalue:", curr_qvalue)
                if abs(curr_qvalue - pre_qvalue) < 0.03:
                    finish_flag = True
                    break
                pre_qvalue = curr_qvalue
            
                isSuccess = (env.distance < 0.5)
                if env.is_incline:
                    print('incline too much, episoide stop')
                    break
                if isSuccess:
                    print('goal reached, episoide stop')
                    break
                
            env.stop_simulation()
            print('i_episoide %d execution finished' %(i_episoide))
            #RL_solver.save_q(weight_path)
            time.sleep(1)
            env.start_simulation()
            if finish_flag:
                print("curr_qvalue:", curr_qvalue)
                break
        
        
    env.stop_simulation()
    env.stop_client()
    #RL_solver.save_q(weight_path)
    
    plt.xlabel("t")
    plt.ylabel("Q value")
    plt.plot(q_value)
    plt.show()
    print ('[INFO] Program ended')

if __name__ == "__main__":
    # execute only if run as a script
    main()
