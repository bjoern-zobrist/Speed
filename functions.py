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
from methods import trust, cob, slsqp, slsqp2
import enum

class method(enum.Enum):
    SLSQP = 0
    TRUST = 1
    COBYLA = 2
    SLSQP2 = 3

def optimize(path, a_pmax, a_pmin, a_smax, v_max, method, start, alphastart, terrain, height):
    

    #COBYLA
    if method == method.COBYLA:

        m = cob(path, a_smax, a_pmax, a_pmin, v_max, start, alphastart, terrain, height)

        #constraints
        cobylacons = m.cons()

        #inital guess
        x0 = m.initial()

        #optimization
        res = minimize(m.fun, x0, method='COBYLA', constraints=cobylacons,options={'maxiter': 1e4, 'disp': True})

    
    #trust-constr
    if method == method.TRUST:
        
        m = trust(path, a_smax, a_pmax, a_pmin, v_max, start, alphastart, terrain, height)

        #constraint
        trustcons = m.cons()
        
        #Boundaries
        bnds = m.bounds()

        #inital guess
        x0 = m.initial()

        #optimization
        res = minimize(m.fun, x0, method='trust-constr', bounds=bnds, options={'maxiter': 1e4, 'disp': True})

    #slsqp
    if method == method.SLSQP:
        
        m = slsqp(path, a_smax, a_pmax, a_pmin, v_max)

        #constraints
        slsqpcons = m.cons()
    
        #Boundaries
        bnds = m.bounds()

        #inital guess
        x0 = m.initial()
    

        #optimization
        res = minimize(m.fun, x0, method='slsqp', bounds=bnds, constraints=slsqpcons)

    if method == method.SLSQP2:

        m = slsqp2(path, a_smax, a_pmax, a_pmin, v_max, start, alphastart, terrain, height)

        #Boundaries
        bnds = m.bounds()

        #constraints
        if start:
            slsqpcons = m.cons()

        #inital guess
        x0 = m.initial()

        #optimization
        if start:
            res = minimize(m.fun, x0, method='slsqp', bounds=bnds, constraints=slsqpcons, tol=1e-10, options={'maxiter': 1e4, 'disp': True}) 
        else:
            res = minimize(m.fun, x0, method='slsqp', bounds=bnds, tol=1e-10, options={'maxiter': 1e4, 'disp': True}) 

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
    track = pd.read_csv("/Users/bjornzobrist/Documents/GitHub/racetrack-database/tracks/Budapest.csv")
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
    #right = np.delete(right,dellist,0)
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
    #left = np.delete(left,dellist,0)
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

#maximum v possible of path
def curvature(path, alpha, a_smax, v_max):
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
    return curvature

#calculate speed possible in path
def speed(pathmax, dx, a_max, a_min, terrain, height):

    if not terrain:
        #if we need to brake
        def brake(v,curve):
            if v[curve]<pathmax[curve]:
                return v
            else:
                v[curve] = pathmax[curve]
                #brake
                while(True):
                    speed = np.sqrt(2*dx[curve-1]*a_min + v[curve]**2)
                    if speed > v[curve-1]:
                        break
                    else:
                        v[curve-1] = speed
                        curve = curve-1
            return v

        v = [] #velocities
        for i in range(len(pathmax)):
            v.append(0)

    
        #find velocities
        for i in range(len(pathmax)):
            if i == 0:
                v[i] = pathmax[i]
            else:
                v[i] = np.sqrt(2*dx[i-1]*a_max + v[i-1]**2)
                v = brake(v,i)

    else:
        #if we need to brake
        def brake(v,curve, height, vcurve, startheight):
            if v[curve]<pathmax[curve]:
                vcurve = vcurve
                startheight = startheight
            else:
                v[curve] = pathmax[curve]
                vcurve = pathmax[curve]
                startheight = height[curve]
                #brake
                while(True):
                    speed = np.sqrt(2*dx[curve-1]*a_min + v[curve]**2)
                    if speed > v[curve-1]:
                        break
                    else:
                        v[curve-1] = speed
                        curve = curve-1
            return v, vcurve, startheight

        v = [] #velocities
        for i in range(len(pathmax)):
            v.append(0)

        startheight = height[0]
        vcurve = 0

        for i in range(len(pathmax)):
            if i == 0:
                v[i] = 0.0
            else:
                g = 9.81
                v[i] = np.sqrt(np.abs(2 * g * (startheight-height[i])) + vcurve**2) 
                v, vcurve, startheight = brake(v,i, height, vcurve, startheight)


    dx = np.array(dx)
    vk = np.delete(np.array(v),0)

    t = np.sum(dx/vk)

    return t,v

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

def plotter(path,position,vel,t,a_p,a_s,distance):
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

    a = int((xr.max()-xr.min())/40+10)
    b = int((yr.max()-yr.min())/40+5)
    fig, axs = plt.subplots(1, 1, figsize=(a,b) ,sharex=True, sharey=True)

    findnorm = np.delete(vel,-1) #because last velocity can't be calculated exactly
    norm = plt.Normalize(findnorm.min(), findnorm.max())
    lc = LineCollection(s, cmap='magma', norm=norm)
    lc.set_array(vel)
    lc.set_linewidth(2)
    line = axs.add_collection(lc)
    fig.colorbar(lc, orientation="horizontal", label="Velocity (m/s)",shrink = 0.5, aspect=30)
    plt.title('Time: %1.3f \n'%t)
    plt.plot(xl,yl,'k')
    plt.plot(xr,yr,'k')
    plt.grid(True)
    plt.xlabel('x [m] \n')
    plt.ylabel('y [m]')
    axs.set_xlim(np.minimum(xr.min(),xl.min())-2, np.maximum(xr.max(),xl.max())+2)
    axs.set_ylim(np.minimum(yr.min(),yl.min())-2, np.maximum(yr.max(),yl.max())+2)
    plt.axis('square')
    fig.savefig('result.png',orientation='portrait')

    x = []
    k = 0
    for i in range(len(vel)):
        x+=[k]
        if i != len(vel)-1:
            k += distance[i]

    plt.rc("font", size=18)
    plt.rc("axes", titlesize=30)
    plt.rc("axes", labelsize=22)
    plt.rc("xtick", labelsize=24)
    plt.rc("ytick", labelsize=24)
    plt.rc("legend", fontsize=22)
    plt.rc("figure", titlesize=24)

    fig1 = plt.figure(figsize=(30,18),dpi=80)
    plt.title('Velocity and Acceleration \n'%t)
    plt.plot(x,vel,'r',label=r'velocity [$m/s$]')
    x = []
    k = 0
    for i in range(len(a_s)):
        x+=[k]
        if i != len(a_s)-1:
            k += distance[i]
    plt.plot(x,a_s,'b',label=r'curve acceleration [$m/s^2$]')
    x = []
    k = 0
    for i in range(len(a_p)):
        x+=[k]
        if i != len(a_p)-1:
            k += distance[i]
    plt.plot(x,a_p,'g',label=r'acceleration [$m/s^2$]')
    plt.grid(True)
    plt.xlabel(r'position [$m$]')
    plt.ylabel('dynamics')
    plt.legend()
    fig1.savefig('result2.png',orientation='portrait')


