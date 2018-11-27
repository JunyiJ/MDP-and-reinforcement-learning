#Introduction

Reinforcement learning is usually applied to solve time-series problems to select the best policy. Specifically, at a certain time slot, the agent interacts with the environment to take an action and receive a reward. A policy is defined by how the agent will act regarding the current state and environment. Reinforcement learning aims to find the best policy which maximizes the reward. There are several commonly used algorithms for reinforcement learning, including value iteration, policy iteration and Q-learning algorithm, SARSA algorithm and so on.

#Problems
In this project, I applied value iteration, policy iteration, Q-learning, SARSA algorithms to two different problems. 
The first problem is an abstraction from a game “Frog cross the river”. The second problem is related to ‘house-keeping robot route planning’ problem.


#Code structure
There are two subfolders within the directory, representing two problems: Frog and robot.
Basically the BURLAP libarary was used to explore MDP and reinforcement learning.