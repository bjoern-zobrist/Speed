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


a_pmax = 30.0 #maximal parallel acceleration
a_pmin = 20.0 #maximal deceleration
a_smax = 10.0 #maximal orthogonal acceleration
v_max = 100.0 #maximal speed
path = fc.track(None,None)
a = 600 #start
b = 1000 #end
method =  method.SLSQP2
sound = True
horizon = True #only with slsqp2

if horizon == True:
    #finite horizon
    horizon = 200
    steps = int(2*(b-a)/horizon)
    alpha = np.array([])
    vel = np.array([])

    for i in range(steps):
        if i == steps-1:
            path = fc.track(int(a+i*horizon/2-1),b)
        else:
            path = fc.track(int(a+i*horizon/2-1),int(a+i*horizon/2+horizon))

        if i == 0:
            res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, None)
        else:
            res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, alpha[-1])

        alphah = res.x
        pathmax = np.array(fc.pmax(path, alphah, a_smax, v_max))
        position = np.array(fc.pos(path, alphah))
        distance = np.array(fc.dist(position))
        t, velh = fc.speed(pathmax, distance, a_pmax, a_pmin)

        if i != 0:
            alpha = np.delete(alpha,-1)
            vel = np.delete(vel,-1)

        if i == steps-1:
            for k in range(len(alphah)):
                alpha = np.append(alpha, alphah[k])
                vel = np.append(vel, velh[k])
        else:
            for k in range(int(horizon/2+1)):
                alpha = np.append(alpha, alphah[k])
                vel = np.append(vel, velh[k])
        print(i)
        print("--- %s seconds ---" % (time.time() - start_time))
        
path = fc.track(a,b)

if horizon == False:
    res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method, None)

print('Calculations done')

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
    t, vel = fc.speed(pathmax, distance, a_pmax, a_pmin)

else:
    #alpha, vel = fc.split(res.x)
    position = np.array(fc.pos(path,alpha))
    t = np.sum(np.array(fc.dist(fc.pos(path,alpha)))/np.delete(np.array(vel),-1))

#calculate from result
a_s = fc.acc(path,alpha,vel,-1)[2]
a_p = fc.acc(path,alpha,vel,-1)[3]

    
fc.plotter(path,position,vel,t)

print('curve\n')
fc.check(a_s,a_smax,0)
print('acc and dec\n')
fc.check(a_p,a_pmax,a_pmin)


print("--- %s seconds ---" % (time.time() - start_time))
