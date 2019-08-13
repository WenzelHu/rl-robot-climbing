import math
import time
import os
import numpy as np
from Vrep.real_world import obstacle, environment
from itertools import cycle, chain
from q.rl_q2 import rl
import _pickle as cPickle
import tqdm
import matplotlib.pyplot as plt
###########################################PREDEFINED VARIABLES#########################################################################################

ACTION_SPACE_LENGTH = 729
STATE_SPACE_LENGTH = 891
NUM_EPISOIDE = 100
READ_WEIGHTS = True
TRAIN_ON_EXP = False
EXP_MAX = 100

##########################################################################################################################################
    

def main():
            
    env = environment()
    env.set_goal(1, 0)
    env.update()
    
    RL_solver = rl(ACTION_SPACE_LENGTH, STATE_SPACE_LENGTH)
    weight_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')

    if not os.path.exists(weight_path):
        print('[ERROR] Weights path %s dose not exist, save weights to current path' % weight_path)
        weight_path = os.getcwd()
    
    if READ_WEIGHTS:
        RL_solver.load_q(weight_path)

    experience_pool = []

    exp_num = 0

    for i_episoide in range(5):
        print('Start episoide %d' %(i_episoide))
        
        env.robot_go_initial()
        env.update()
        RL_solver.rl_init(env.states)
        
        for t in range(1000):
            if env.goal_position[0] == 0:
                #return
                env.action = RL_solver.action_prev
                if env.states < 880:
                    RL_solver.tempture = 2
                else:
                    RL_solver.tempture = 0.5
                env.take_action()
                env.update()
                RL_solver.make_policy()
                isSuccess = (env.distance < 0.05)
                if isSuccess:
                    print('goal reached, episoide ends')
                    break
            else:
                exp_num = exp_num + 1
                env.action = RL_solver.action_prev
                #env.action = 252
                curr_experience = [env.states, env.action]
                if env.states < 880:
                   RL_solver.tempture = 2
                else:
                   RL_solver.tempture = 0.5
                env.take_action()

                env.update()
                reward = env.get_reward()
            
                RL_solver.q_learning(env.states, reward)
                #RL_solver.action_prev = 252
                curr_experience += [reward,env.states,RL_solver.action_prev]
                experience_pool.append(curr_experience)
                isSuccess = (env.distance < 0.05)
                if isSuccess:
                    print('goal reached, episoide stop')
                    break

                print("exp_len: ",len(experience_pool))
                print("current sarsa: ",curr_experience)

                if exp_num >= EXP_MAX:
                    break
                
        print('episoide %d execution finished' %(i_episoide))

        if exp_num >= EXP_MAX:
            file_path = os.path.join(weight_path, "experience.pkl")
            with open(file_path, 'wb') as writefile:
                cPickle.dump(experience_pool, writefile)
                print('[INFO] Sucessfully save experience to %s' % (file_path))
            break
        
        if isSuccess:
            print('Current goal reached, change goal alternately')
            if env.goal_position[0] == 1:
                env.set_goal(0,0)
            else:
                env.set_goal(1,0)

    
        #RL_solver.save_q(weight_path)


    print("collecting finished!")
    '''
    file_path = os.path.join(fold_path, "experience.pkl")
    with open(file_path, 'rb') as readfile:
        experience_pool = cPickle.load(readfile)
    '''
    '''
    file_path = os.path.join(weight_path, "experience.pkl")
        if not os.path.exists(file_path):
            print('[ERROR] The q file %s does not exist' % (weight_path))
            return
        else:
            with open(file_path, 'rb') as readfile:
                experience_pool = cPickle.load(readfile)
            print('[INFO] Sucessfully load q file from %s' % (file_path))
    '''
    '''
    RL_solver = rl(ACTION_SPACE_LENGTH, STATE_SPACE_LENGTH)
    if READ_WEIGHTS:
        RL_solver.load_q(weight_path)
    
    if TRAIN_ON_EXP:
        print("exp num: ",len(experience_pool))
        error = []
        for _ in tqdm.trange(1000):
            for idx,sarsa in enumerate(experience_pool):
                RL_solver.state_prev = sarsa[0]
                RL_solver.action_prev = sarsa[1]
                reward = sarsa[2]
                states_ = sarsa[3]
                RL_solver.action_aft_policy  = sarsa[4]
                q_ = RL_solver.q
                RL_solver.q_learning1(states_, reward)
                error.append(np.sum(np.abs(q_-RL_solver.q)))
        
        RL_solver.save_q(weight_path)
        print("total error",sum(error))
        plt.plot(error)
        plt.show()
    '''
    print ('[INFO] Program ended')

if __name__ == "__main__":
    # execute only if run as a script
    main()
