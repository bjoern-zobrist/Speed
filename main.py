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


a_pmax = 15.0 #maximal parallel acceleration
a_pmin = 30.0 #maximal deceleration
a_smax = 40.0 #maximal orthogonal acceleration
v_max = 83.0 #maximal speed
path = fc.track(None,None)
a = 690 #start
b = 730 #end
method =  method.SLSQP2
sound = False
horizon = False #only with slsqp2
terrain = False #only with slsqp2
height = []
if terrain:
    for i in range(b-a):
        height.append(np.sqrt((b-a)-i)*10)
        #height.append(b-a-i)


if horizon:
    #finite horizon
    horizon = 10
    steps = int(10*(b-a-horizon)/horizon)
    alpha = np.array([])
    vel = np.array([])

    for i in range(steps):
        if i == steps-1:
            path = fc.track(int(a+i*horizon/10),b)
        else:
            path = fc.track(int(a+i*horizon/10),int(a+i*horizon/10+horizon))

        if i == 0:
            res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, False, None, terrain, height)
        else:
            res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, True, alpha[len(alpha)-int(horizon/10):], terrain, height)

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


if not horizon:
    res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, False, None, terrain, height)

if sound:
    Datei = '455602__inspectorj__tripod-horn-blast-single-01.wav'
    wave = sa.WaveObject.from_wave_file(Datei)
    play = wave.play()
    play.wait_done()


if method == method.SLSQP2 or method == method.TRUST or method == method.COBYLA:
    if not horizon:
        alpha = res.x
    pathmax = np.array(fc.pmax(path, alpha, a_smax, v_max))
    position = np.array(fc.pos(path, alpha))
    distance = np.array(fc.dist(position))
    t, vel = fc.speed(pathmax, distance, a_pmax, a_pmin, terrain, height)

else:
    alpha, vel = fc.split(res.x)
    position = np.array(fc.pos(path,alpha))
    t = np.sum(np.array(fc.dist(fc.pos(path,alpha)))/np.delete(np.array(vel),-1))

#calculate from result
a_s = fc.acc(path,alpha,vel,-1)[2]
a_p = fc.acc(path,alpha,vel,-1)[3]

curvature = np.array(fc.curvature(path, alpha, a_smax, v_max))
    
fc.plotter(path,position,vel,t,a_p,a_s,distance)


print("--- %s seconds ---" % (time.time() - start_time))
