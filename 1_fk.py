#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 20 19:06:56 2019

@author: JoseMa
"""

import sys
sys.path.append('./lib')
from fk import m1_fk
import numpy as np
from math import pi

fk = m1_fk()
    
'''INPUTS'''
# joint positions
q = [0.1, 0.0, pi/2, 0.0]
# End effector offset/orientation: x, y, z, rx, ry, rz
xt = 0.01
yt = 0.01
zt = 0.05
rxt = 0
ryt = 0
rzt = 0
''''''

q_res = input('Joint states (radians) in the format: q1 q2 q3 q4 [0.1  0.0  3.14/2  0.0]: ')
ee_res = input('End effector translation (m) and rotation (rad) in format: x y z rx ry rz: [0.01 0.01 0.05 0.0 0.0 0.0]: ')
print('*'*40)
if q_res != '':
    q = np.fromstring(q_res, dtype=float, sep=' ')
if ee_res != '':
    xt, yt, zt, rxt, ryt, rzt = np.fromstring(ee_res, dtype=float, sep=' ')



fk.set_ee(xt, yt, zt, rxt, ryt, rzt)
    
poses = fk.get_A(q)

print('*'*40)
fk.print_ref_frames(poses)

print('*'*40)
print('Solution using homogeneous transformations FK:')
print(np.round(fk.fk_using_A(q), decimals=3))
print('Solution using the analytical FK:')
print(np.round(fk.fk_analytic(q), decimals=3))
print('*'*40 + '\n')