#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 15:53:20 2021

@author: bjornzobrist
"""

#if you go slow you die

import functions as fc
import numpy as np
import time
import simpleaudio as sa
from functions import method

start_time = time.time()


a_pmax = 20.0 #maximal parallel acceleration
a_pmin = 20.0 #maximal deceleration
a_smax = 10.0 #maximal orthogonal acceleration
v_max = np.inf #maximal speed
path = fc.track(None,None)
a = 700 #start
b = 850 #end
method =  method.SLSQP2
sound = True
horizon = True #only with slsqp2
terrain = False #only with slsqp2
if terrain == True:
    a_pmax = []
    for i in range(b-a):
        a_pmax.append((b-a-i)/5)

if horizon == True:
    #finite horizon
    horizon = 100
    steps = int(10*(b-a-horizon)/horizon)
    alpha = np.array([])
    vel = np.array([])

    for i in range(steps):
        if i == steps-1:
            path = fc.track(int(a+i*horizon/10),b)
        else:
            path = fc.track(int(a+i*horizon/10),int(a+i*horizon/10+horizon))

        if i == 0:
            res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, False, None, terrain)
        else:
            res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, True, alpha[len(alpha)-int(horizon/10):], terrain)

        alphah = res.x

        if i == steps-1:
            alpha = np.append(alpha, alphah[int(horizon/10):])

        if i == 0:
            alpha = np.append(alpha, alphah[0:2*int(horizon/10)])

        else:
            alpha = np.append(alpha, alphah[int(horizon/10):2*int(horizon/10)])
            
        print(i+1, "of", steps)
        print("--- %s seconds ---" % (time.time() - start_time))
        
path = fc.track(a,b)


if horizon == False:
    res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, False, None, terrain)

if sound == True:
    Datei = '455602__inspectorj__tripod-horn-blast-single-01.wav'
    wave = sa.WaveObject.from_wave_file(Datei)
    play = wave.play()
    play.wait_done()


if method == method.SLSQP2:
    if horizon == False:
        alpha = res.x
    pathmax = np.array(fc.pmax(path, alpha, a_smax, v_max))
    position = np.array(fc.pos(path, alpha))
    distance = np.array(fc.dist(position))
    t, vel = fc.speed(pathmax, distance, a_pmax, a_pmin, terrain)

else:
    #alpha, vel = fc.split(res.x)
    position = np.array(fc.pos(path,alpha))
    t = np.sum(np.array(fc.dist(fc.pos(path,alpha)))/np.delete(np.array(vel),-1))

#calculate from result
a_s = fc.acc(path,alpha,vel,-1)[2]
a_p = fc.acc(path,alpha,vel,-1)[3]

    
fc.plotter(path,position,vel,t)


print("--- %s seconds ---" % (time.time() - start_time))
