import math
import time
import os
import numpy as np
from Vrep.real_world import obstacle, environment
from itertools import cycle, chain
from q.rl_q2 import rl
###########################################PREDEFINED VARIABLES#########################################################################################

ACTION_SPACE_LENGTH = 729
STATE_SPACE_LENGTH = 891
NUM_EPISOIDE = 100
READ_WEIGHTS = True


##########################################################################################################################################
    

def main():
            
    env = environment()
    env.set_goal(0.9, 0)
    env.update()
    
    RL_solver = rl(ACTION_SPACE_LENGTH, STATE_SPACE_LENGTH)
    RL_solver.set_temperature(1.0)
    weight_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')
    if not os.path.exists(weight_path):
        print('[ERROR] Weights path %s dose not exist, save weights to current path' % weight_path)
        weight_path = os.getcwd()
    
    if READ_WEIGHTS:
        RL_solver.load_q(weight_path)
    
    for i_episoide in range(NUM_EPISOIDE):
        print('Start episoide %d' %(i_episoide))
        
        env.robot_go_initial()
        env.update()
        env.prev_distance = env.distance
        RL_solver.rl_init(env.states)
        
        for t in range(1000):
            env.action = RL_solver.action_prev
            #env.action = 252
            env.take_action()
            env.update()
            reward = env.get_reward()
            if RL_solver.state_prev >= 880 or env.goal_position[0] == 0:
                RL_solver.state_aft = env.states
                RL_solver.make_policy_aft()
                RL_solver.state_prev = RL_solver.state_aft
                RL_solver.action_prev = RL_solver.action_aft_policy
            else:
                RL_solver.q_learning(env.states, reward)
            
            isSuccess = (env.distance < 0.05)
            if isSuccess:
                print('goal reached, episoide stop')
                break
                
        print('episoide %d execution finished' %(i_episoide))
        
        if isSuccess:
            print('Current goal reached, change goal alternately')
            if env.goal_position[0] == 0.9:
                env.set_goal(0,0)
            else:
                env.set_goal(0.9,0)
        RL_solver.save_q(weight_path)
        
        
    #RL_solver.save_q(weight_path)

    print ('[INFO] Program ended')

if __name__ == "__main__":
    # execute only if run as a script
    main()
XT