#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 15:53:20 2021

@author: bjornzobrist
"""

import functions as fc
import time
import numpy as np
import matplotlib.pyplot as plt

start_time = time.time()

N = 101 #discretization
a_max = 1 #max acceleration
a_min = 1 #min acceleration
dx = [] #distance between points
'''
pathmax = [ 50.25772609,  48.9675181 ,  57.26484467 , 74.14295747 , 71.7571502,
  70.1454735,   67.58130738 , 64.35071864,  61.44155563 , 58.73259999,
  55.65134248 , 52.22031401,  48.58180458,  44.91979835,  42.1622999,
  41.58629227,  43.40906443,  46.33038534,  49.25738601,  50.89344395,
  52.44342444,  90.70495142,  51.16805139 , 37.50706462,  33.82912881,
  29.1688319,   25.07643593,  24.02584654,  26.93537569,  31.74655822,
  35.76431685,  39.24102806,  42.69004083 , 46.11726525 , 48.84220282,
  51.19393931 , 53.69210168,  56.21585634,  58.70677637,  77.97441862,
 111.35168689 , 65.13595744,  67.14066655 , 68.48517248 , 67.30700445,
  71.34049132 , 74.20258428,  76.07388821 , 87.78686277, 106.83023925] #maximum speed on path
'''
pathmax = []
v = [] #velocities
a = [] #acceleration

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


for i in range(N):
    dx.append(1)
    v.append(0)
    a.append(0)
    if i == 60:
        pathmax.append(0.5)
    else:
        pathmax.append(np.inf)
    

for i in range(N):
    if i == 0:
        v[i] = 0.0
    else:
        v[i] = np.sqrt(2*dx[i-1]*a_max + v[i-1]**2)
        v = brake(v,i)
    


for i in range(N-1):
    a[i] = ((v[i+1])**2 - (v[i])**2)/(2*dx[i])


dx = np.array(dx)
t = np.sum(dx[1:]/(v[1:]))

print("--- %s seconds ---" % (time.time() - start_time))

print(t)


x = []
for i in range(len(v)):
    x+=[i]

plt.rc("font", size=18)
plt.rc("axes", titlesize=38)
plt.rc("axes", labelsize=30)
plt.rc("xtick", labelsize=30)
plt.rc("ytick", labelsize=30)
plt.rc("legend", fontsize=30)
plt.rc("figure", titlesize=28)

plt.rc('text', usetex=True)
plt.rc('font', family='serif')
fig = plt.figure(figsize=(30,18),dpi=80)
plt.title('MV, Time: %1.3f \n'%t)
plt.plot(x,v,'r',label=r'velocity [$m/s$]')
plt.plot(x,a,'b',label=r'acceleration [$m/s^2$]')
plt.plot(x,pathmax,'g',label=r'maximum velocity [$m/s$]')
plt.grid(True)
plt.xlabel('position')
plt.ylabel('dynamics')
plt.legend()
fig.savefig('result.png',orientation='portrait')


