from q.rl_qfa_1 import rl
from numpy import *
import tqdm

ACTION_LEFT  = 0
ACTION_NONE  = 1
ACTION_RIGHT = 2
STATE_LEFT_OF_LINE = 0
STATE_ON_LINE = 1
STATE_RIGHT_OF_LINE = 2
STATES = [STATE_LEFT_OF_LINE,STATE_ON_LINE,STATE_RIGHT_OF_LINE]

def f(x, u):

	if x == STATE_ON_LINE:
		if u == ACTION_LEFT:
			result = [0.2, 0.7, 0.1]
		elif u == ACTION_RIGHT:
			result = [0.1, 0.7, 0.2]
		elif u == ACTION_NONE:
			result = [0.0, 1.0, 0.0]

	elif x == STATE_LEFT_OF_LINE:
		if u == ACTION_LEFT:
			result = [1.0,0.0,0.0]
		elif u == ACTION_RIGHT:
			result = [0.0,1.0,0.0]
		elif u == ACTION_NONE:
			result = [1.0, 0.0, 0.0]

	elif x == STATE_RIGHT_OF_LINE:
		if u == ACTION_LEFT:
			result = [0.0,1.0,0.0]
		elif u == ACTION_RIGHT:
			result = [0.0,0.0,1.0]
		elif u == ACTION_NONE:
			result = [0.0, 0.0, 1.0]

	return result

def getNextState(s,u):
	if s == STATE_ON_LINE:
		if u == ACTION_LEFT:
			nextState = random.choice(STATES,1,True,[0.2, 0.7, 0.1])
		elif u == ACTION_RIGHT:
			nextState = random.choice(STATES,1,True,[0.1, 0.7, 0.2])
		elif u == ACTION_NONE:
			nextState = random.choice(STATES,1,True,[0.0, 1.0, 0.0])

	elif s == STATE_LEFT_OF_LINE:
		if u == ACTION_LEFT:
			nextState = random.choice(STATES,1,True,[1.0,0.0,0.0])
		elif u == ACTION_RIGHT:
			nextState = random.choice(STATES,1,True,[0.0,1.0,0.0])
		elif u == ACTION_NONE:
			nextState = random.choice(STATES,1,True,[1.0,0.0,0.0])

	elif s == STATE_RIGHT_OF_LINE:
		if u == ACTION_LEFT:
			nextState = random.choice(STATES,1,True,[0.0,1.0,0.0])
		elif u == ACTION_RIGHT:
			nextState = random.choice(STATES,1,True,[0.0,0.0,1.0])
		elif u == ACTION_NONE:
			nextState = random.choice(STATES,1,True,[0.0,0.0,1.0])

	return int(nextState)

def getFeature(s):
	if s==STATE_ON_LINE:
		return [0,1,0]
	if s==STATE_LEFT_OF_LINE:
		return [1,0,0]
	if s==STATE_RIGHT_OF_LINE:
		return [0,0,1]

def r(s):
	if s==1:
		return 1
	else:
		return 0

nA = 3
nS = 3
rl_solver = rl(len_action_space = nA, len_feature = nS)

s = STATE_ON_LINE
rl_solver.rl_init(getFeature(s))

for idx in tqdm.trange(10000):
	a = rl_solver.make_policy()
	s_ = getNextState(s,a)
	features = getFeature(s_)
	reward = r(s_)
	rl_solver.q_learning(features, reward)
	s = s_
'''
rl_solver.q_learning([1,0,0], 0)
print(rl_solver.make_policy())
rl_solver.q_learning([0,1,0], 1)
print(rl_solver.make_policy())
rl_solver.q_learning([0,0,1], 0)
print(rl_solver.make_policy())
'''

w = rl_solver.getWeights()
Phi = mat([[1,0,0],[0,1,0],[0,0,1]])
print (Phi*w)