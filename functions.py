#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 20:18:03 2021

@author: bjornzobrist
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize
from numpy.linalg import norm
import pandas as pd

#rotate 90 degrees
def rot(x):
    theta =  np.pi/2
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])
    rot = np.dot(R, x)
    return rot


#distance between the points
def dist(x):
    dist = []
    for i in range(len(x)-1):
        a = x[i]
        b = x[i+1]
        dist.append(np.sqrt( (np.abs(b[0]-a[0]))**2 + (np.abs(b[1]-a[1]))**2 ))
    return dist

#calculate position with alpha
def pos(path,alpha):
    position = []
    left = path[0]
    right = path[1]
    for i in range(len(right)):
        gate = [alpha[i]*(left[i,0]-right[i,0]),alpha[i]*(left[i,1]-right[i,1])]
        position.append([right[i,0]+gate[0],right[i,1]+gate[1]])
    return position

#calculate velocity
def vel(v,position):
    vel = []
    for i in range(len(position)-1):
        velocity = position[i+1]-position[i]
        vel.append( v * velocity/norm(velocity))
    return vel


#calculate acceleration
def acc(path,alpha,v,i):
    left,right = path
    left = left[i:i+3] #take parts we need
    right = right[i:i+3]
    path = left,right
    
    position = np.array(pos(path, alpha))
    velocity = vel(v,position)
    distance = np.array(dist(position))
    time = distance/v
    dv = velocity[1]-velocity[0]
    a = norm([dv[0]/time[0],dv[1]/time[0]])
    return a

#constraints
def constraints(path,v):
    #max curve acc
    def constraint_maker(i=0):  # i MUST be an optional keyword argument, else it will not work
        def constraint(x):
           return  - acc(path,[x[i],x[i+1],x[i+2]],v,i) + 1
        return constraint
    
    #begin in middle of track
    c=[{'type': 'eq', 'fun': lambda x:  x[0] - 0.5}]
    
    #add max and min acc
    for i in range(len(path[0])-2):
        c+=[{'type': 'ineq', 'fun': constraint_maker(i)}]
    
    return c

# Boundaries
def bounds(k):
    b = []
    for i in range(k):
        b+=[(0, 1)]
    return b

def track():
    '''
    #straight line
    left = np.array([[0,1],[1,1],[2,1],[3,1],[4,1]])
    right = np.array([[0,0],[1,0],[2,0],[3,0],[4,0]])
    
    #diagonal line
    left = np.array([[0,1],[1,2],[2,3],[3,4]])
    right = np.array([[1,0],[2,1],[3,2],[4,3]])

    #curve
    left = np.array([[0,1],[1,1],[1.5,1],[2,1],[2.25,1.25],[2.5,1.5],[2.6,2],[2.6,2.5],[2.5,3],[2,3.5],[1.5,3.5],[1,3.5],[0,3.5]])
    right = np.array([[0,0],[1,0],[1.5,0],[2,0],[2.25,0.25],[2.5,0.5],[3,1.5],[3,3],[2.75,4],[2,4.5],[1.5,4.5],[1,4.5],[0,4.5]])
    '''
    #track
    track = pd.read_csv("/Users/bjornzobrist/Documents/GitHub/racetrack-database/tracks/Silverstone.csv")
    track = track.to_numpy()

    #extract middle line
    middle = []
    for i in range(len(track)):
        middle.append([track[i,0],track[i,1]])
    middle = np.array(middle)    
    
    #calculate the direction
    direction = []
    for i in range(len(middle)-1):
        direction.append([middle[i,0]-middle[i+1,0],middle[i,1]-middle[i+1,1]])
    direction.append(direction[-1])

    #find right side
    right = []
    for i in range(len(middle)):
        perp = rot(direction[i]) #the right angled vector
        perp = perp/norm(perp) #norm
        right.append(middle[i]+track[i,2]*perp)
    right = np.array(right)
    right = right[750:800]

    #find left side
    left = []
    for i in range(len(middle)):
        perp = rot(direction[i]) #the right angled vector
        perp = perp/norm(perp) #norm
        left.append(middle[i]-track[i,3]*perp)
    left = np.array(left)
    left = left[750:800]
    
    return left,right

def plotter(path,position):
    left = path[0]
    right = path[1]
    xl = left[:,0]
    yl = left[:,1]
    xr = right[:,0]
    yr = right[:,1]
    xp = position[:,0]
    yp = position[:,1]
    
    plt.rcParams.update({'font.size':22})
    fig1 = plt.figure(figsize=(30,18),dpi=80)
    plt.title('Speed \n')
    plt.plot(xl,yl,'r',label='track')
    plt.plot(xr,yr,'r')
    plt.plot(xp,yp,'b',label='race line')
    plt.grid(True)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    fig1.savefig('result.png',orientation='portrait')

def optimize(v):
    #track
    path = track()
    
 
    #function to minimize
    fun = lambda x: np.sum(dist(pos(path,x)))

    #constraints
    cons = tuple(constraints(path,v))
    
    #initial guess
    x0 = []
    for i in range(len(path[0])):
        x0.append(0.5)
    
    #Boundaries
    bnds = tuple(bounds(len(x0)))
    

    #optimization
    res = minimize(fun, x0, method='SLSQP', bounds=bnds, constraints=cons)
    alpha = res.x
    position = np.array(pos(path,alpha))
    a =[]
    for i in range(len(path[0])-2):
        #take the correct alphas
        q = alpha[i]
        w = alpha[i+1]
        e = alpha[i+2]
        a.append(acc(path,[q,w,e],v,i))
    
    time = np.sum(dist(pos(path,alpha)))/v
    
    plotter(path,position)
    
    return time