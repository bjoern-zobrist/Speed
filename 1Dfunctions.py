#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 20:18:03 2021

@author: bjornzobrist
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize, NonlinearConstraint, Bounds



# Constraints for slqp
def constraints(k, curves, v_max, a, dx):
    #max acc
    def constraint_maker1(i=0,v_max=v_max, a_max = a[1]):  # i MUST be an optional keyword argument, else it will not work
        def constraint1(x):
            if v_max == None:
                return -((x[i+1])**2 - (x[i])**2)/(2*dx) + a_max
            else:
                return  -((x[i+1])**2 - (x[i])**2)/(2*dx) + a_max * (1-(x[i]/v_max)**2)
        return constraint1
    
    #min acc
    def constraint_maker2(i=0, a_min = a[0]):  # i MUST be an optional keyword argument, else it will not work
        def constraint2(x):
            return  ((x[i+1])**2 - (x[i])**2)/(2*dx) - a_min 
        return constraint2
    
    #curve max v
    def constraint_maker3(i=0,k=k):  # i MUST be an optional keyword argument, else it will not work
        def constraint3(x):
            return  -x[i[0]+k]+i[2]
        return constraint3
    
    
    #begin with velocity 0
    c=[{'type': 'eq', 'fun': lambda x:  x[0]}]
    
    #add max and min acc
    for i in range(k):
        c+=[{'type': 'ineq', 'fun': constraint_maker1(i)}]
        c+=[{'type': 'ineq', 'fun': constraint_maker2(i)}]
        
    #add curves
    for i in curves:
        for k in range(i[1]-i[0]):
            c+=[{'type': 'ineq', 'fun': constraint_maker3(i,k)}]
    return c


#constraints for trust-constr
class cons:
    def __init__(self, n, dx, curves):
        self.n = n
        self.dx = dx
        self.curves = curves
    def get_cons(self, x):
        #begin with velocity 0
        result = [x[0]]
        #add acc
        for i in range(self.n):
            result.append(((x[i+1])**2 - (x[i])**2)/(2*self.dx))
        #add curves
        for i in self.curves:
            for k in range(i[1]-i[0]):
                result.append(x[i[0]+k])
        return result


# Boundaries for slqp
def bounds(k,v_max):
    b = []
    for i in range(k):
        b+=[(0, v_max)]
    return b




#initial guess
def guess(k):
    x = []
    for i in range(k):
        x.append(0.1)
    return x


# Plot
def plotter(v,a,t):
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
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
    fig1 = plt.figure(figsize=(30,18),dpi=80)
    plt.title('SLSQP, Time: %1.3f \n'%t)
    plt.plot(x,v,'r',label=r'velocity [$m/s$]')
    plt.plot(x,a,'b',label=r'acceleration [$m/s^2$]')
    plt.grid(True)
    plt.xlabel(r'position [$m$]')
    plt.ylabel('dynamics')
    plt.legend()
    fig1.savefig('result.png',orientation='portrait')
    
def optimize(N, x0, curves, v_max, a, dx):
    #function 
    fun = lambda x: np.sum(dx/x[1:])

    '''
    #trust-constr
    #constraints
    cmin = [0]
    cmax = [0]
    for i in range(N-1):
        cmin.append(a[0])
        cmax.append(a[1])
    for i in curves:
        for k in range(i[1]-i[0]):
            cmin.append(-np.inf)
            cmax.append(i[2])
    c = cons(N-1,dx,curves)
    constraints = NonlinearConstraint(c.get_cons, cmin, cmax ,keep_feasible=True)
        
    #Boundaries
    bmin = []
    bmax = []
    for i in range(N):
        bmin.append(0)
        bmax.append(v_max)
    bnds = Bounds(bmin, bmax)
    '''
    
    #slsqp
    #constraints
    cons = tuple(constraints(N-1,curves,v_max, a, dx))
        
    #Boundaries
    bnds = tuple(bounds(N,v_max))
    

    #initial guess
    if x0 == None:
        x0 = tuple(guess(N))
    else:
        x0 = x0
    
    #optimization
    res = minimize(fun, x0, method='slsqp', bounds=bnds, constraints=cons, options={'maxiter': 1e4, 'disp': True})
    
    #extract results
    velocity = res.x
    
    acceleration = [0]
    for i in range(len(velocity)-1):
        acceleration += [((velocity[i+1])**2 - (velocity[i])**2)/(2*dx)]

    time = np.sum(dx/(velocity[1:]))

    print(time)

    plotter(velocity,acceleration,time)
    
    return acceleration

