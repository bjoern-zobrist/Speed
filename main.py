#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 15:53:20 2021

@author: bjornzobrist
"""

#if you go slow you die

import functions as fc
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import time

start_time = time.time()


a_pmax = 5 #maximal parallel acceleration
a_smax = 5 #maximal orthogonal acceleration

res = fc.optimize(a_pmax, a_smax)

print('Calculations done')

path = fc.track()
alpha, vel = fc.split(res.x)
position = np.array(fc.pos(path,alpha))
a_s = []
a_p = []
for i in range(len(path[0])-2):
    #take the correct alphas
    q = alpha[i]
    w = alpha[i+1]
    e = alpha[i+2]
    a_s.append(fc.acc(path,[q,w,e],[vel[i],vel[i+1]],i)[0])
    a_p.append(fc.acc(path,[q,w,e],[vel[i],vel[i+1]],i)[1])
    
t = np.sum(np.array(fc.dist(fc.pos(path,alpha)))/np.array(vel))
    
fc.plotter(path,position,vel)

print(t)
print(a_p)
print(a_s)
print(alpha)
print(vel)

print("--- %s seconds ---" % (time.time() - start_time))
















