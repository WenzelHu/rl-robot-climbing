import math
import time
import os
import numpy as np
from Vrep.real_world import obstacle, environment
from itertools import cycle, chain
from q.rl_q0 import rl
import _pickle as cPickle
import tqdm
import matplotlib.pyplot as plt

ACTION_SPACE_LENGTH = 729
STATE_SPACE_LENGTH = 891
NUM_EPISOIDE = 100
READ_WEIGHTS = True
TRAIN_ON_EXP = True
EXP_MAX = 10

weight_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')

file_path = os.path.join(weight_path, "experience.pkl")
if not os.path.exists(file_path):
    print('[ERROR] The q file %s does not exist' % (weight_path))
else:
    with open(file_path, 'rb') as readfile:
        experience_pool = cPickle.load(readfile)
    print('[INFO] Sucessfully load q file from %s' % (file_path))

RL_solver = rl(ACTION_SPACE_LENGTH, STATE_SPACE_LENGTH)
if READ_WEIGHTS:
    RL_solver.load_q(weight_path)
    
if TRAIN_ON_EXP:
    print("exp num: ",len(experience_pool))
    error = []
    #del experience_pool[0]
    for _ in tqdm.trange(100):
        np.random.shuffle(experience_pool)
        for idx,sarsa in enumerate(experience_pool): 
            #print(sarsa)
            RL_solver.state_prev = sarsa[0]
            RL_solver.action_prev = sarsa[1]
            reward = sarsa[2]
            states_ = sarsa[3]
            #RL_solver.action_aft_policy  = sarsa[4]
            q_ = RL_solver.q.copy()
            #print("q_: ",q_)
            RL_solver.q_learning(states_, reward)
            #print("q: ",RL_solver.q)
            error.append(np.sum(np.abs(q_-RL_solver.q)))
        #RL_solver.e = np.zeros((STATE_SPACE_LENGTH, ACTION_SPACE_LENGTH))
        
    RL_solver.save_q(weight_path)
    print("total error",sum(error))
    plt.plot(error)
    plt.grid()
    plt.title("Convergence Curve of Q table")
    plt.xlabel("iteration")
    plt.ylabel("change")
    plt.show()