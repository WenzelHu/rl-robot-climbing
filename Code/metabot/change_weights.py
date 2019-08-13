import os
import numpy as np
import _pickle as cPickle

states_num = 880
fold_path = os.path.join(os.getcwd(), os.path.pardir, os.path.pardir, 'Weights')
file_path = os.path.join(fold_path, "q.pkl")
if not os.path.exists(file_path):
    print('[ERROR] The q file %s does not exist' % (fold_path))
else:
    with open(file_path, 'rb') as readfile:
        q = cPickle.load(readfile)
    print('[INFO] Sucessfully load q file from %s' % (file_path))
    q[880:891,:] = q[states_num, :]
    
    #q_ = q[states_num, :]
    #q_max = q_.max()
    #q_index = np.where(q_ == q_max)
    #print("weight for state %d is:" % (states_num), q_)
    #print("length:", len(q_))
    #print("max:", q_max)
    #print("index:", q_index)
    #print(q.shape)
    #print(q_[252])
    with open(file_path, 'wb') as writefile:
        cPickle.dump(q, writefile)
        print('[INFO] Sucessfully save q file to %s' % (file_path))
