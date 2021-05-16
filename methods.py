#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 20:18:03 2021

@author: bjornzobrist
"""
import functions as fc
from scipy.optimize import minimize, NonlinearConstraint, Bounds
import numpy as np

#trust-constr
class trust:
    def __init__(self, path, a_smax, a_pmax, a_pmin, v_max):
        self.path = path
        self.a_smax = a_smax
        self.a_pmax = a_pmax
        self.a_pmin = a_pmin
        self.v_max = v_max

    def fun(self, x):
        alpha, vel = fc.split(x)
        vel = np.array(vel)
        for i in range(len(vel)):
            if vel[i] == 0:
                vel[i] = 0.1
        f = np.sum(np.array(fc.dist(fc.pos(self.path,alpha)))/vel)
        return f 

    def get_cons(self, x):
        result = []
        #curve acc
        for i in range(len(self.path[0])-2):
            result.append(fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[0])
        #acc
        for i in range(len(self.path[0])-2):
            result.append(fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[1])
        return result

    def cons(self):
        #constraints
        cmin = []
        cmax = []
        #cons of curve acc
        for i in range(len(self.path[0])-2):
            cmin.append(0.0)
            cmax.append(self.a_smax)
        for i in range(len(self.path[0])-2):
            cmin.append(-self.a_pmin)
            cmax.append(self.a_pmax)
        cons = NonlinearConstraint(self.get_cons, cmin, cmax ,keep_feasible=True)
        return cons

    def bounds(self):
        #Boundaries
        bmin = []
        bmax = []
        #bounds of alpha
        for i in range(len(self.path[0])):
            bmin.append(0.0)
            bmax.append(1.0)
        #bounds of speed
        for i in range(len(self.path[0])):
            bmin.append(0.0)
            bmax.append(self.v_max)
        bnds = Bounds(bmin, bmax)
        return bnds


#COBYLA
class cob:
    def __init__(self, path, a_smax, a_pmax, a_pmin, v_max):
        self.path = path
        self.a_smax = a_smax
        self.a_pmax = a_pmax
        self.a_pmin = a_pmin
        self.v_max = v_max

    def fun(self, x):
        alpha, vel = fc.split(x)
        vel = np.array(vel)
        for i in range(len(vel)):
            if vel[i] == 0:
                vel[i] = 0.1
        f = np.sum(np.array(fc.dist(fc.pos(self.path,alpha)))/vel)
        return f 

    #constraints
    def cons(self):
        #max curve acc
        def constraint_maker1(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint1(x):
                return  - fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[0] + self.a_smax
            return constraint1

        #max acc
        def constraint_maker2(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint2(x):
                return  - fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[1] + self.a_pmax
            return constraint2

        #min acc
        def constraint_maker3(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint3(x):
                return   fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[1] + self.a_pmin
            return constraint3

        #max alpha
        def constraint_maker4(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint4(x):
                return  - x[i] + 1
            return constraint4

        #min alpha
        def constraint_maker5(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint5(x):
                return   x[i]
            return constraint5

        #max v
        def constraint_maker6(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint6(x):
                return  - x[len(self.path[0])+i] + self.v_max
            return constraint6

        #min v
        def constraint_maker7(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint7(x):
                return   x[len(self.path[0])+i] 
            return constraint7
    
        c = []
    
        #add constraints
        for i in range(len(self.path[0])-2):
            c+=[{'type': 'ineq', 'fun': constraint_maker1(i)}]
            c+=[{'type': 'ineq', 'fun': constraint_maker2(i)}]
            c+=[{'type': 'ineq', 'fun': constraint_maker3(i)}]
    
        for i in range(len(self.path[0])):
            c+=[{'type': 'ineq', 'fun': constraint_maker4(i)}]
            c+=[{'type': 'ineq', 'fun': constraint_maker5(i)}]
            c+=[{'type': 'ineq', 'fun': constraint_maker6(i)}]
            c+=[{'type': 'ineq', 'fun': constraint_maker7(i)}]
    
        return tuple(c)

#slsqp
class slsqp:
    def __init__(self, path, a_smax, a_pmax, a_pmin, v_max):
        self.path = path
        self.a_smax = a_smax
        self.a_pmax = a_pmax
        self.a_pmin = a_pmin
        self.v_max = v_max

    def fun(self, x):
        alpha, vel = fc.split(x)
        vel = np.array(vel)
        for i in range(len(vel)):
            if vel[i] == 0:
                vel[i] = 0.1
        f = np.sum(np.array(fc.dist(fc.pos(self.path,alpha)))/vel)
        
        #penalty for constraints
        g = []
        for i in range(len(self.path[0])-2):
            #take the correct alphas
            q = alpha[i]
            w = alpha[i+1]
            e = alpha[i+2]
            #curve acc
            g.append(fc.acc(self.path,[q,w,e],[vel[i],vel[i+1]],i)[0]-self.a_smax)
        penalty = 0
        for i in range(len(g)):
            if g[i] > 0:
                penalty += vel[i]*0.07
        
        return f + penalty

    #constraints
    def cons(self):
        #max curve acc
        def constraint_maker1(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint1(x):
                return  - fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[0] + self.a_smax
            return constraint1

        #max acc
        def constraint_maker2(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint2(x):
                return  - fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[1] + self.a_pmax
            return constraint2

        #min acc
        def constraint_maker3(i=0):  # i MUST be an optional keyword argument, else it will not work
            def constraint3(x):
                return   fc.acc(self.path,[x[i],x[i+1],x[i+2]],[x[len(self.path[0])+i],x[len(self.path[0])+i+1]],i)[1] + self.a_pmin
            return constraint3
    
        c = []
    
        #add constraints
        for i in range(len(self.path[0])-2):
            c+=[{'type': 'ineq', 'fun': constraint_maker1(i)}]
            c+=[{'type': 'ineq', 'fun': constraint_maker2(i)}]
            c+=[{'type': 'ineq', 'fun': constraint_maker3(i)}]
            
        return tuple(c)

    # Boundaries
    def bounds(self):
        b = []
        for i in range(len(self.path[0])*2):
            if i < int(len(self.path[0])+1):
                b+=[(0, 1)]
            else:
                b+=[(0,self.v_max)]
        return tuple(b)