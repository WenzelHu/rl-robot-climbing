import math
import time
import os
import numpy as np
from Vrep.vrep_simulation import obstacle, environment
from itertools import cycle, chain
from q.rl_q2 import rl
import _pickle as cPickle
import tqdm
###########################################PREDEFINED VARIABLES#########################################################################################

ACTION_SPACE_LENGTH = 729
STATE_SPACE_LENGTH = 891
NUM_EPISOIDE = 100
READ_WEIGHTS = True
TRAIN_ON_EXP = False
EXP_MAX = 10

##########################################################################################################################################
    

def main():
            
    env = environment()
    env.start_simulation()
    env.set_goal(-2, -2)
    env.update()

    experience_pool = []
    exp_pool_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'experience')
    exp_num = 0

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
        RL_solver.rl_init(env.states)
        
        for t in range(1000):
            env.action = RL_solver.action_prev
            curr_experience = [env.states, env.action]
            env.take_action()
            env.update()
            reward = env.get_reward()
            RL_solver.state_aft = env.states
            RL_solver.make_policy_aft()
            RL_solver.q_learning(env.states, reward)

            curr_experience += [reward,env.states,RL_solver.action_aft_policy]
            experience_pool.append(curr_experience)
            exp_num += 1
            '''
            if RL_solver.state_prev == 880:
                RL_solver.state_aft = env.states
                RL_solver.make_policy_aft()
                RL_solver.state_prev = RL_solver.state_aft
                RL_solver.action_prev = RL_solver.action_aft_policy
            else:
            '''
            
            isSuccess = (env.distance < 0.05)
            if env.is_incline:
                print('incline too much, episoide stop')
                break
            if isSuccess:
                print('goal reached, episoide stop')
                break

            print("exp_len: ",len(experience_pool))
            print("current sarsa: ",curr_experience)

            if exp_num >= EXP_MAX:
                break

        if exp_num >= EXP_MAX:
            file_path = os.path.join(weight_path, "experience.pkl")
            with open(file_path, 'wb') as writefile:
                cPickle.dump(experience_pool, writefile)
                print('[INFO] Sucessfully save experience to %s' % (file_path))
            break
                
        env.stop_simulation()
        print('episoide %d execution finished' %(i_episoide))

        RL_solver.save_q(weight_path)
        time.sleep(1)
        env.start_simulation()
        
    env.stop_simulation()
    env.stop_client()


    RL_solver = rl(ACTION_SPACE_LENGTH, STATE_SPACE_LENGTH)
    
    if TRAIN_ON_EXP:
        for _ in tqdm.trange(10000):
            for idx,sarsa in enumerate(experience_pool):
                RL_solver.state_prev = sarsa[0]
                RL_solver.action_prev = sarsa[1]
                reward = sarsa[2]
                states_ = sarsa[3]
                RL_solver.action_aft_policy  = sarsa[4]
                RL_solver.q_learning(states_, reward)


        RL_solver.save_q(weight_path)

    print ('[INFO] Program ended')

if __name__ == "__main__":
    # execute only if run as a script
    main()
