#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 20 19:12:04 2019

@author: JoseMa
"""
import sys
sys.path.append('./lib')
from profiles import timing_law
from plot_tools import plot_pvaj
from math import pi
'''
INPUTS
'''
t_cycle = 0.01
p_i = pi # initial position
p_f = -pi # final position

V = 2 # Velocity (unit/sec)
Vmax = 2 # Velocity (unit/sec)
A = 4 # Acceleration (unit/sec2)
T = 4 # Duration
''''''
print('*'*40)
print('\tINPUTS')
print('*'*40)
p_i_res = input('Initial joint state (rad) [-3.14]: ')
p_f_res = input('Final  joint state (rad) [3.14]: ')
V_res = input('Target velocity (in rad/s): [2]')
Vmax_res = input('Maximum velocity (in rad/s): [2]')
A_res = input('Target Acceleration (in rad/s2) [4]')
T_res = input('Desired trajectory time (in seconds) [10]: ')

print('*'*40)
if p_i_res != '':
    p_i = float(p_i_res)
if p_f_res != '':
    p_f = float(p_f_res)
if V_res != '':
    V = float(V_res)
if Vmax_res != '':
    Vmax = float(Vmax_res)
if A_res != '':
    A = float(A_res)
if T_res != '':
    T = float(T_res)
    
''' MAIN '''

timing = timing_law(p_i, p_f, t_cycle, T, V, Vmax, A)

#timing.generate_trapezoidal_profile()
timing.generate_st_profile()

plot_pvaj(timing.p, timing.v, timing.a, timing.j, vertical=True, title='ST Profile')
