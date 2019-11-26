#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 20 19:08:38 2019

@author: JoseMa
"""
import sys
sys.path.append('./lib')
from fk import m1_fk
from ik import m1_ik
import numpy as np
from math import pi



''' INPUTS '''
# End effector rotation/translation
xt = 0.01
yt = 0.0
zt = 0.05
rxt = 0
ryt = 0
rzt = 0

# Initial position/rotation
p_i = np.asarray([0.2, 0.1, 0.1, 0, 0, 0.7], dtype=np.float32)
# Target  position/rotation
p_f = np.asarray([0.1, 0.2, 0.05, 0, 0, -0.7], dtype=np.float32)

''''''

pi_res = input('Starting position (m) and orientation (rad) in format: x y z rx ry rz [0.2 0.1 0.1 0 0 0.7]: ')
pf_res = input('Endposition (m) and orientation (rad) in format: x y z rx ry rz [0.1 0.2 0.0.5 0 0 -0.7]: ')
ee_res = input('End effector translation and rotation in format: x y z rx ry rz [0.01 0 0.05 0 0 0]: ')
print('\n'+'*'*40)
if pi_res != '':
    p_i = np.fromstring(pi_res, dtype=float, sep=' ')
if pf_res != '':
    p_f = np.fromstring(pf_res, dtype=float, sep=' ')
if ee_res != '':
    xt, yt, zt, rxt, ryt, rzt = np.fromstring(ee_res, dtype=float, sep=' ')


''' MAIN '''
ik = m1_ik()
# Set EE configuration
ik.set_ee(xt, yt, zt, rxt, ryt, rzt)

rdot_des = p_i - p_f
print('p_dot desired: {}'.format(rdot_des))

'''
ANALYTICAL RESULT
[Jacobian for differential inverse kinematics is also available in ik.py]
'''
print('*'*40)
px, py, pz, rx, ry, rz = p_i
ik_closed_i = np.asarray(ik.closed_ik(px, py, pz, rx, ry, rz))
for i, solution in enumerate(ik_closed_i):
    print('Initial Joint States {}: {}'.format(i, solution))
print('*'*40)

px, py, pz, rx, ry, rz = p_f
ik_closed_f = np.asarray(ik.closed_ik(px, py, pz, rx, ry, rz))
for i, solution in enumerate(ik_closed_f):
    print('Final Joint States {}: {}'.format(i, solution))
print('*'*40) 

q_displacement = ik_closed_f - ik_closed_i
for i, solution in enumerate(q_displacement):
    print('Analytical IK {}: {}'.format(i, solution))
    
