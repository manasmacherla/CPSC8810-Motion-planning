            # agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


import numpy as np
from math import sqrt
import random
import math

class Agent(object):


    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent
        # self.xpos = int(csvParameters[2])
        # self.rad = self.radius*self.radius



    def computeNewVelocity(self, neighbors=[]):
        """ 
            Your code to comp
            Compute the new velocity of the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
        Distance = []
        Distance1 = []
        for agent in neighbors:
            if not agent.id == self.id:
                Dist1 = sqrt(np.square(agent.pos[0] - self.pos[0]) + np.square(agent.pos[1] - self.pos[1]))
                Dist = self.pos - agent.pos
                Distance1.append(Dist1)
                Distance.append(Dist)

################ sampling Vcand values ##############################################################################
        no_samples = 100
        A = np.zeros((no_samples, 2))

        for x in range(0, no_samples):
            r = random.uniform(0, self.maxspeed*self.maxspeed)
            t = random.uniform(0, 2 * math.pi)
            X = sqrt(r) * math.cos(t)
            Y = sqrt(r) * math.sin(t)
            X = round(X, 3)
            Y = round(Y, 3)
            A[x][0] = X
            A[x][1] = Y # matrix A represents the list of candid velocities

        A_vel = []
        for agent in neighbors:
            if not agent.id == self.id:
                D = agent.vel
                A_vel.append(D)
#                else:
#                    break
        # print(A_vel)
        
    ### Calculating the cost function for each candidate velocity
        cost_func = []
        alp = 1
        beta = 1
        gama = 2
        for i in range (len(A)):
            Time = []
            for j in range (len(A_vel)):
                rad = 2*self.radius
                w = Distance[j]
                C = np.dot(w, w) - rad*rad
                if C <= 0:
                    T = 0
                elif C > 0:
                    V = A[i] - A_vel[j]
                    a = np.dot(V, V)
                    b = np.dot(w, V)
                    if b > 0:
                        T = 10000000000000000
                    elif b <= 0:
                        discr = b*b - a*C
                        if discr <= 0:
                            T = 10000000000000000
                        elif discr > 0:
                            tau = C/(-b + sqrt(discr))
                            if tau < 0:
                                T = 10000000000000000
                            elif tau >= 0:
                                T = tau
                Time.append(T)
            T1 = min(Time)
            alp_1 = A[i] - self.gvel
            alp_1 = np.dot(alp_1, alp_1)
            alp_1 = sqrt(alp_1)

            beta_1 = A[i] - self.vel
            beta_1 = np.dot(beta_1, beta_1)
            beta_1 = sqrt(beta_1)

            K = alp*(alp_1) + beta*(beta_1) + (gama/T1)
            cost_func.append(K)

        K1 = min(cost_func)
        index = cost_func.index(K1)
        self.vnew[:] = A[index][:]

        # if not self.atGoal:
        # self.vnew[:] = self.gvel[:]   # here I just set the new velocity to be the goal velocity


    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed


            
            
  