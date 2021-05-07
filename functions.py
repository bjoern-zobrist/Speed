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
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

#rotate 90 degrees
def rot(x):
    theta =  np.pi/2
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)]])
    rot = np.dot(R, x)
    return rot

#normieren
def one(a):
    a = np.array(a)/norm(np.array(a))
    return a

#splits vector in two parts
def split(x):
    alpha = []
    vel = []
    for i in range(len(x)):
        if i < int(len(x)/2+1):
            alpha.append(x[i])
        else:
            vel.append(x[i])
    return alpha, vel


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

#check constraints
def check(a, max, min):
    #delete first and last element, as they can't be calculated correctly
    a = np.delete(a,-1)
    a = np.delete(a,0)
    #tolerance
    epsilon = 0.6
    for i in range(len(a)):
        if max - a[i] < -epsilon:
            print('Accident!!!')
        if min + a[i] < -epsilon:
            print('Accident!!!')

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
    velocity = np.array(vel(v,position)) #direction of velocity
    v = np.array(v) #speed
    distance = np.array(dist(position))
    time = distance/v
    dv = v[1]*velocity[1]-v[0]*velocity[0]
    a = np.array([dv[0]/time[0],dv[1]/time[0]])
    a_s = norm(a - (np.dot(a,velocity[0]))/(np.dot(velocity[0],velocity[0])) * velocity[0]) #orthogonal part to velocity
    a_p = (np.dot(a,velocity[0]))/(np.dot(velocity[0],velocity[0])) * velocity[0] #parallel part of a to velocity
    if v[1] > v[0]:
        a_p = norm(a_p)
    else:
        a_p = -norm(a_p)
    return a_s, a_p

#constraints
def constraints(path, a_smax, a_pmax, a_pmin, half):
    #max curve acc
    def constraint_maker1(i=0):  # i MUST be an optional keyword argument, else it will not work
        def constraint1(x):
           return  - acc(path,[x[i],x[i+1],x[i+2]],[x[half+i],x[half+i+1]],i)[0] + a_smax
        return constraint1

    #max acc
    def constraint_maker2(i=0):  # i MUST be an optional keyword argument, else it will not work
        def constraint2(x):
           return  - acc(path,[x[i],x[i+1],x[i+2]],[x[half+i],x[half+i+1]],i)[1] + a_pmax
        return constraint2

    #min acc
    def constraint_maker3(i=0):  # i MUST be an optional keyword argument, else it will not work
        def constraint3(x):
           return   acc(path,[x[i],x[i+1],x[i+2]],[x[half+i],x[half+i+1]],i)[1] + a_pmin
        return constraint3
    
    c = []
    
    #add constraints
    for i in range(len(path[0])-2):
        c+=[{'type': 'ineq', 'fun': constraint_maker1(i)}]
        c+=[{'type': 'ineq', 'fun': constraint_maker2(i)}]
        c+=[{'type': 'ineq', 'fun': constraint_maker3(i)}]
    
    
    return c

# Boundaries
def bounds(k,v_max):
    b = []
    for i in range(k):
        if i < int(k/2+1):
            b+=[(0, 1)]
        else:
            b+=[(0,v_max)]
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

def plotter(path,position,vel,t):
    left = path[0]
    right = path[1]
    xl = left[:,0]
    yl = left[:,1]
    xr = right[:,0]
    yr = right[:,1]
    xp = np.array(position[:,0])
    yp = np.array(position[:,1])
    vel = np.array(vel)

    
    p = np.array([xp, yp]).T.reshape(-1, 1, 2)
    s = np.concatenate([p[:-1], p[1:]], axis=1)

    fig, axs = plt.subplots(1, 1, sharex=True, sharey=True)

    findnorm = np.delete(vel,-1) #because last velocity can't be calculated exactly
    norm = plt.Normalize(findnorm.min(), findnorm.max())
    lc = LineCollection(s, cmap='magma', norm=norm)
    lc.set_array(vel)
    lc.set_linewidth(2)
    line = axs.add_collection(lc)
    fig.colorbar(line, ax=axs)
    plt.title('Time: %1.3f \n'%t)
    plt.plot(xl,yl,'k')
    plt.plot(xr,yr,'k')

    plt.grid(True)
    plt.xlabel('x')
    plt.ylabel('y')
    axs.set_xlim(np.minimum(xr.min(),xl.min())-2, np.maximum(xr.max(),xl.max())+2)
    axs.set_ylim(np.minimum(yr.min(),yl.min())-2, np.maximum(yr.max(),yl.max())+2)
    fig.savefig('result.png',orientation='portrait')


def optimize(a_pmax, a_pmin, a_smax, v_max):
    #track
    path = track()

    half = int(len(path[0])) #the first half of x ist alpha, the second is v, half is to find this part
    
 
    #function to minimize
    def fun(x):
        alpha, vel = split(x)
        vel = np.array(vel)
        f = np.sum(np.array(dist(pos(path,alpha)))/vel)
        return f

    #constraints
    cons = tuple(constraints(path, a_smax, a_pmax, a_pmin, half))
    
    #initial guess
    x0 = []
    for i in range(len(path[0])):
        x0.append(0.5)
    for i in range (len(path[0])):
        x0.append(10)
    
    #Boundaries
    bnds = tuple(bounds(len(x0),v_max))
    

    #optimization
    res = minimize(fun, x0, method='SLSQP', bounds=bnds, constraints=cons)

    return res