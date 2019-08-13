import os
import numpy as np
import _pickle as cPickle

class rl():  # with eligibility traces
	def __init__(self, len_action_space, len_state_space):  ### need to change later
		self.len_action_space = len_action_space
		self.len_state_space = len_state_space
		self.temperature = 2.0
		self.discount_factor = 0.90
		self.alpha = 0.01
		self._lambda = 0.1
		self.q = np.random.random_sample((self.len_state_space, self.len_action_space))
		self.e = np.zeros((self.len_state_space, self.len_action_space))
		self.state_prev = None  # should be the initial state!!!! change later
		self.action_prev = None
		self.state_aft = None
		self.action_aft_policy = None
		self.action_atf_greedy = None

	def rl_init(self, state):
		state = np.array(state)
		self.state_prev = state

	def softmax(self, x):
		x = x / self.temperature
		exp_x = np.exp(x)
		softmax_x = exp_x / np.sum(exp_x)
		return softmax_x

	def make_policy(self):
		"""
		Creates an policy based on given Q-values.
		Returns: action to pick, note that the return value is the index of the action in action_space
		"""
		softmax_predictions = self.softmax(self.state_prev)
		self.action_prev = np.random.choice(range(len(softmax_predictions)), 1, p=softmax_predictions.tolist())[0]
		return self.action_prev  # this is the index of the action in action_space

	def make_policy_aft(self):
		"""
		Creates an policy based on given Q-values.
		Returns: action to pick, note that the return value is the index of the action in action_space
		"""
		softmax_predictions = self.softmax(self.state_aft)
		self.action_aft_policy = np.random.choice(range(len(softmax_predictions)), 1, p=softmax_predictions.tolist())[0]
		return self.action_aft_policy  # this is the index of the action in action_space

	def q_learning(self, next_state, reward):
		#best_action = make_policy(self.q[state,:])
		#excute action, get reward and the next state
		self.state_aft = next_state
		self.make_policy_aft()
		self.action_atf_greedy = np.argmax(self.q[self.state_aft,:])
		# one step in the q learning, mainly the TD update
		mask = reward + self.discount_factor * self.q[self.state_aft,self.action_aft_policy] - self.q[self.state_prev,self.action_atf_greedy]
		self.e[self.state_prev,self.action_prev] += 1
		# for all state and action pairs
		self.q += self.alpha * mask * self.e
		if self.action_aft_policy == self.action_atf_greedy:
			self.e = self.discount_factor * self._lambda * self.e
		else:
			self.e = np.zeros((self.len_state_space, self.len_action_space))
		self.state_prev = self.state_aft
		self.action_prev = self.action_aft_policy

	def save_q(self, fold_path):
		if not os.path.exists(fold_path):
			print('[ERROR] The output path %s does not exist' % (fold_path))
			return
		else:
			file_path = os.path.join(fold_path, "q.pkl")
			with open(file_path, 'wb') as writefile:
				cPickle.dump(self.q, writefile)
			print('[INFO] Sucessfully save q file to %s' % (file_path))

	def load_q(self, fold_path):
		file_path = os.path.join(fold_path, "q.pkl")
		if not os.path.exists(file_path):
			print('[ERROR] The q file %s does not exist' % (fold_path))
			return
		else:
			with open(file_path, 'rb') as readfile:
				self.q = cPickle.load(readfile)
			print('[INFO] Sucessfully load q file from %s' % (file_path))