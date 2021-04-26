#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 20:18:03 2021

@author: bjornzobrist
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

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


#constraints
def constraints():
    #begin in middle of track
    c=[{'type': 'eq', 'fun': lambda x:  x[0] - 0.5}]
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
    '''
    #diagonal line
    left = np.array([[0,1],[1,2],[2,3],[3,4]])
    right = np.array([[1,0],[2,1],[3,2],[4,3]])
    return left,right

def optimize():
    path = track()
    
 
    #function to minimize
    fun = lambda x: np.sum(dist(pos(path,x)))

    #constraints
    cons = tuple(constraints())
    
    #initial guess
    x0 = []
    for i in range(len(path[0])):
        x0.append(0.3)
    
    #Boundaries
    bnds = tuple(bounds(len(x0)))
    

    #optimization
    res = minimize(fun, x0, method='SLSQP', bounds=bnds, constraints=cons)
    return res