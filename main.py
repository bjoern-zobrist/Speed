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
method =  method.SLSQP2
sound = True

path = fc.track(700,900)

res = fc.optimize(path, a_pmax, a_pmin, a_smax, v_max, method)

print('Calculations done')

if sound == True:
    Datei = '455602__inspectorj__tripod-horn-blast-single-01.wav'
    wave = sa.WaveObject.from_wave_file(Datei)
    play = wave.play()
    play.wait_done()


if method == method.SLSQP2:
    alpha = res.x
    pathmax = np.array(fc.pmax(path, alpha, a_smax, v_max))
    position = np.array(fc.pos(path, alpha))
    distance = np.array(fc.dist(position))
    t, vel = fc.speed(pathmax, distance, a_pmax, a_pmin)

else:
    alpha, vel = fc.split(res.x)
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
