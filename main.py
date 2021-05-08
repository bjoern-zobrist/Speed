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


a_pmax = 8 #maximal parallel acceleration
a_pmin = 12 #maximal deceleration
a_smax = 8 #maximal orthogonal acceleration
v_max = 20 #maximal speed
path = fc.track(375,425)

res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max)

print('Calculations done')


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
    
fc.plotter(path,position,vel,t)

fc.check(a_s,a_smax,0)
print(a_s)
fc.check(a_p,a_pmax,a_pmin)
print(a_p)



print("--- %s seconds ---" % (time.time() - start_time))
