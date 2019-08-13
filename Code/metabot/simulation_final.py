import math
import time
import os
from Vrep.vrep_simulation import obstacle, environment
from itertools import cycle, chain
from q.rl_qfa_1 import rl
###########################################PREDEFINED VARIABLES#########################################################################################

ACTION_SPACE_LENGTH = 108
NUM_EPISOIDE = 10000
READ_WEIGHTS = False

obstacles = list()
obstacle1 = obstacle([4,4],0,2,2,0)
obstacles.append(obstacle1)
obstacle2 = obstacle([-4,-4],0,2,2,0)
obstacles.append(obstacle2)

##########################################################################################################################################
    

def main():
            
    env = environment()
    env.start_simulation()
    env.update()

    RL_solver = rl(ACTION_SPACE_LENGTH, len(env.get_features()))
    
    if READ_WEIGHTS:
        weight_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')
        RL_solver.load_weights(weight_path)
        
        
    for i_episoide in range(NUM_EPISOIDE):
        print('Start episoide %d' %(i_episoide))

    	# at beginning of each episoide, let robot go to home pose  # get current joint position and robot postion and robot orientation
        env.robot_go_initial()
        RL_solver.rl_init(env.features)
        
        starttime=time.time()
        env.update()
        for t in range(10000):
            env.action = RL_solver.make_policy()
            env.take_action()
            #print(time.time() - starttime)
            time.sleep(0.05 - ((time.time() - starttime) % 0.05))
            starttime=time.time()
            env.update()
            
            isSuccess = (env.distance < 0.002)
            isFail = (t==9999)
            reward = env.get_reward2()
            #print(reward)
            RL_solver.q_learning(env.features, reward)
            if abs(env.curr_robot_orientation[0]) > 3 or abs(env.curr_robot_orientation[1]) > 3:
                print('incline too much, episoide stop')
                break
            if isSuccess:
                print('goal reached, episoide stop')
                break
            if isFail:
                print('fail to reach goal, episoide end')
        
        env.stop_simulation()
        print('episoide %d execution finished' %(i_episoide))

        if i_episoide % 500 == 499:
            RL_solver.save_weights(weight_path)
        time.sleep(1)
        env.start_simulation()
    

    #print ('body position', get_body_position_streaming(clientID, Body_Handle))
    env.stop_simulation()
    env.stop_client()
    
    weight_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')
    if not os.path.exists(weight_path):
        print('[ERROR] Weights path %s dose not exist, save weights to current path' % weight_path)
        weight_path = os.getcwd()
    RL_solver.save_weights(weight_path)
        
    
    print ('[INFO] Program ended')

if __name__ == "__main__":
    # execute only if run as a script
    main()
