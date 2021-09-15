#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 15:53:20 2021

@author: bjornzobrist
"""

import functions as fc
import time
import numpy as np

start_time = time.time()

#inputs
N = 101 #discretization
x0 = None #initial guess
curves = [[20,41,8.0]] #place of curve: per curve [start, end, v_max]
v_max = np.inf #maximum velocity
a = [-1,1] #acceleration bounds
dx = 1 #delta x

result = fc.optimize(N, x0, curves, v_max, a, dx)



print("--- %s seconds ---" % (time.time() - start_time))