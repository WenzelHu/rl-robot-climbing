# Reinforcement learning for metabot climbing obstacle

## Overview

In order to build a RL agent for climbing task on the robot, we need to define a state space, a action space and reward function, such that RL agent is able to know to control the robot to do which action under a certain state via trainning. Besides, states defined in the space space need to be observable via sensors, in our case, we only have a camera on a smartphone looking at the scenario from above. Thus, states need to be some visual information that can be captured by camera. Interface, thourgh with the RL agent is able to send the control message to robot's motors, is implemented.

