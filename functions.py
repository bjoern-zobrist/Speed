#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 20:18:03 2021

@author: bjornzobrist
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize, NonlinearConstraint, Bounds
from numpy.linalg import norm
import pandas as pd
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm
from methods import trust, cob, slsqp
import enum

class method(enum.Enum):
    SLSQP = 0
    TRUST = 1
    COBYLA = 2

def optimize(path, a_pmax, a_pmin, a_smax, v_max, method):
    

    #initial guess
    x0 = []
    for i in range(len(path[0])):
        x0.append(0.5)
    for i in range (len(path[0])):
        x0.append(1.0)

    #COBYLA
    if method == method.COBYLA:

        m = cob(path, a_smax, a_pmax, a_pmin, v_max)

        #constraints
        cobylacons = m.cons()

        #optimization
        res = minimize(m.fun, x0, method='COBYLA', constraints=cobylacons)

    
    #trust-constr
    if method == method.TRUST:
        
        m = trust(path, a_smax, a_pmax, a_pmin, v_max)

        #constraint
        trustcons = m.cons()
        
        #Boundaries
        bnds = m.bounds()

        #optimization
        res = minimize(m.fun, x0, method='trust-constr', bounds=bnds, constraints=trustcons)

    #slsqp
    if method == method.SLSQP:
        
        m = slsqp(path, a_smax, a_pmax, a_pmin, v_max)

        #constraints
        slqpcons = m.cons()
    
        #Boundaries
        bnds = m.bounds()
    

        #optimization
        res = minimize(m.fun, x0, method='slsqp', bounds=bnds, constraints=slqpcons)

    return res




def track(a,b):
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
    #delete every second point
    dellist = []
    zerolist = []
    for i in range(len(right)):
        if i%2 != 0:
            dellist.append(i)
            zerolist.append(0)
    right = np.delete(right,dellist,0)
    #select part of racetrack
    if a == None:
        right = right
    else:
        right = right[a:b]
    

    #find left side
    left = []
    for i in range(len(middle)):
        perp = rot(direction[i]) #the right angled vector
        perp = perp/norm(perp) #norm
        left.append(middle[i]-track[i,3]*perp)
    left = np.array(left)
    left = np.delete(left,dellist,0)
    if a == None:
        left = left
    else:
        left = left[a:b]
    
    
    return left,right

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

def acc(path,alpha,v,i):
    if i != -1:
        left,right = path
        left = left[i:i+3] #take parts we need
        right = right[i:i+3]
        path = left,right

    position = np.array(pos(path, alpha))
    v = np.array(v) #speed
    distance = np.array(dist(position))

    #calculate curve acc
    xt = np.gradient(position[:,0])
    yt = np.gradient(position[:,1])
    velocity = np.array([ [xt[i], yt[i]] for i in range(len(xt))])
    speed = np.sqrt(xt**2+yt**2)

    #curvature
    xtt = np.gradient(xt)
    ytt = np.gradient(yt)
    curvature = np.abs(xtt * yt - xt * ytt) / (speed**2)**1.5
    
    a_s = v**2 * curvature

    #calculate acc and dec
    a_p = []
    for i in range(len(v)-1):
        a_p.append(((v[i+1])**2 - (v[i])**2)/(2*distance[i]))

    return a_s[1], a_p[1], a_s, a_p

#maximum v possible of path
def pmax(path, alpha, a_smax, v_max):
    position = np.array(pos(path, alpha))
    xt = np.gradient(position[:,0])
    yt = np.gradient(position[:,1])
    velocity = np.array([ [xt[i], yt[i]] for i in range(len(xt))])
    speed = np.sqrt(xt**2+yt**2)

    #curvature
    xtt = np.gradient(xt)
    ytt = np.gradient(yt)

    curvature = np.abs(xtt * yt - xt * ytt) / (speed**2)**1.5
    v = np.sqrt(a_smax/curvature) #maximum possible speed
    #if it's higher than vmax
    for i in range(len(v)):
        if v[i]>v_max:
            v[i] = v_max
    return v


#check constraints
def check(a, max, min):
    a = np.delete(a,0)
    #tolerance
    epsilon = 0.2
    for i in range(len(a)):
        if max - a[i] < -epsilon:
            print('Accident!!!')
            print(i)
        if min + a[i] < -epsilon:
            print('Accident!!!')
            print(i)


#splits vector in two parts
def split(x):
    alpha = []
    vel = []
    for i in range(len(x)):
        if i < int(len(x)/2):
            alpha.append(x[i])
        else:
            vel.append(x[i])
    return alpha, vel

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


