#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 16 14:15:05 2019

@author: josema
"""
import numpy as np
from matplotlib import pyplot as plt
from math import pi, radians

'''
v1: speed at start
vn: speed at end
qs: passing points
hs: spline durations
'''

class splines(object):
    def __init__(self, v_i, v_f, points, T, t_cycle, vmax, debug=False):
        self.debug=debug
        self.xs = []
        self.ys = []
        self.v1 = v_i # Initial velocitiy
        self.vn = v_f # Final velocity
        self.vmax = vmax
        self.T = T
        for p in points:
            self.xs.append(p[0]) # X
            self.ys.append(p[1]) # Y
        self.trajectory = []
        
        self.n_points = len(self.ys)
        self.hs = []
        
        # Get spline durations
        for i in range(0, len(self.xs) - 1):
            self.hs.append(self.xs[i+1] - self.xs[i])#1/(self.n_points-1))
        
        self.t_cycle = t_cycle
            
        self.spline_coef = [] # The spline coefficientes for cubic polynomial
        
        self.n_splines = len(self.hs)
        self.generate_coefficientes_algorithm()
        
        self.x = []
        self.y = []
        self.ydot = []
        self.yddot = []
        
        self.generate_spline()
    

    '''
    Function to calculate the velocities in the middle knots
    Velocity calculation extracted from: http://www.diag.uniroma1.it/deluca/rob1_en/13_TrajectoryPlanningJoints.pdf
    '''
    def calculate_mid_velocities(self):
        self.vs = np.empty(len(self.ys), dtype = np.float32)
        self.vs[0] = self.v1
        self.vs[-1] = self.vn
            
        
        A = np.zeros((self.n_splines-1, self.n_splines-1), dtype=np.float32)
        b = np.zeros(self.n_splines-1, dtype=np.float32)
        
        
        # Create coeficient and b matrices for A*v = b
        for i in range(self.n_splines-1):
            if i == 0:
                A[i,0] = 2*(self.hs[0]+self.hs[1])
                A[i,1] = self.hs[0]
                b[i] = (3/(self.hs[0]*self.hs[1]))*( (self.ys[2]-self.ys[1])*(self.hs[0]**2) + (self.ys[1]-self.ys[0])*(self.hs[1]**2)) - self.hs[1]*self.v1
                
            elif i == self.n_splines-2:
                A[i,-1] = 2*(self.hs[i]+self.hs[i+1])
                A[i,-2] = self.hs[i+1]
                b[i] = (3/(self.hs[i]*self.hs[i+1]))*( (self.hs[i]**2)*(self.ys[i+2]-self.ys[i+1]) + (self.hs[i+1]**2)*(self.ys[i+1]-self.ys[i]) ) - self.hs[i]*self.vn
            else:
                A[i,i-1] = self.hs[i+1]
                A[i,i] = 2*(self.hs[i]+self.hs[i+1])
                A[i,i+1] = self.hs[i]
                b[i] = (3/(self.hs[i]*self.hs[i+1]))*( (self.ys[i+2]-self.ys[i+1])*(self.hs[i]**2) + (self.ys[i+1]-self.ys[i])*(self.hs[i+1]**2))
        self.A = A 
        self.b = b
        A_inv = np.linalg.inv(A)
        
        # Get the velocities
        self.vs[1:len(self.ys)-1] = np.matmul(A_inv, b)
        if self.debug:
            print('Velocities: {}'.format(self.vs))
        
    '''
    Spline a2 and a3 coefficient calculation extracted from: http://mathworld.wolfram.com/CubicSpline.html
    (the algorithm in the Sapienza slides didn't work)
    '''    
    def generate_coefficientes_algorithm(self):
        self.spline_coef = []
        self.calculate_mid_velocities()
        m = self.T/self.n_splines
        for i in range(self.n_splines):
            a0 = self.ys[i]
            a1 = self.vs[i]*self.hs[i]
            a2 = 3*(self.ys[i+1] - self.ys[i]) - 2*self.vs[i]*self.hs[i] - self.vs[i+1]*self.hs[i]
            a3 = 2*(self.ys[i] - self.ys[i+1]) + self.vs[i]*self.hs[i] + self.vs[i+1]*self.hs[i]
       
            # Sapienza calculation doesn't work
#            H = np.asarray([[self.hs[i]**2, self.hs[i]**3], 
#                            [2*self.hs[i], 3*self.hs[i]**2]], dtype=np.float32)
#            Q = np.asarray([[self.ys[i+1] - self.ys[i] - self.vs[i]*self.hs[i]],
#                            [self.vs[i+1] - self.vs[i]]], dtype=np.float32)
#            a2, a3 = np.matmul(np.linalg.inv(H), Q)
            
            self.spline_coef.append([a0, a1, float(a2), float(a3)])
        
        self.spline_coef = np.array(self.spline_coef)
        if self.debug:
            print('Splines are now ready')

    
    '''
    Get position for spline i at time tao
    Tao = [0,1]
    '''
    def generate_spline(self):
        steps = 0
        tref = 0
        for i in range(self.n_splines):
            if self.debug:
                print('Go from {} to {}'.format(self.ys[i], self.ys[i+1]))
            tref += self.xs[i]
            ref = self.xs[i]
            a = self.spline_coef[i]
            # CHECK CONTINUITY
#            tao = 0
#            print('Start q: {}'.format(a[0] + a[1]*tao + a[2]*(tao**2) + a[3]*(tao**3)))
#            print('Start v: {}'.format(a[1] + 2*a[2]*(tao) + 3*a[3]*(tao**2)))
#            print('Start a: {}'.format(a[2] + 6*a[3]*(tao)))
#            tao = 1
#            print('End q: {}'.format(a[0] + a[1]*tao + a[2]*(tao**2) + a[3]*(tao**3)))
#            print('End v {}:'.format(a[1] + 2*a[2]*(tao) + 3*a[3]*(tao**2)))
#            print('End a: {}'.format(a[2] + 6*a[3]*(tao)))
#            print('*'*20)
            

            for tao in np.arange(0, 1, self.t_cycle):
                self.x.append(ref + tao*(self.hs[i]))
                self.y.append(a[0] + a[1]*tao + a[2]*(tao**2) + a[3]*(tao**3))
                self.ydot.append((a[1] + 2*a[2]*(tao) + 3*a[3]*(tao**2))/self.hs[i])
                self.yddot.append((2*a[2] + 6*a[3]*(tao))/self.hs[i]**2)
                self.trajectory.append((self.x[-1], self.y[-1]))
        return self.trajectory



if __name__ == '__main__':
    t_cycle = 0.01
    T = 14
    qmax = [0.25, pi/2, radians(140), 2*pi]
    
#    points = [(-0.09, -0.05), (-0.05, 0.02), (0.01, -0.01), (0.09, 0.06)]
#    points = [(1,45),(2,90),(2.5,-45),(4,45)]
    points = [[ 0.0, -4.137],
    [ 4.569, 1.163],
    [ 8.264, -2.339],
    [15.0, -0.847]]
    vi = 0#(x[1]-x[0])/0.25
    vf = 0#(x[3]-x[2])/0.5
    s = splines(vi, vf, points, T, t_cycle, qmax)
    

    plt.plot(s.x, s.y)
    plt.show()
#    plt.plot(np.arange(len(s.x)), s.ydot)
    plt.plot(s.x, s.ydot)
    plt.show()
    plt.plot(s.x, s.yddot)
    plt.show()
    print(s.spline_coef)
    
    
    
        
    
    
        
    
    