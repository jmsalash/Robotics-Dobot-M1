#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 14 12:04:40 2019

@author: JoseMa
"""

import sys
sys.path.append('./lib')
from fk import m1_fk
from ik import m1_ik
from profiles import timing_law
from lib.plot_tools import plot_pvaj

from math import radians, pi
import numpy as np

'''
JOINT AND VEL/ACC LIMITS DOBOT M1
'''
qmin = [0, -pi/2, -radians(140), -2*pi]
qmax = [0.25, pi/2, radians(140), 2*pi]
qdot_max = [1, pi, pi, pi] # [m, rad, rad, rad]
acc_multiplier = 5 # No specs found about acceleration
qddot_max = [1*acc_multiplier, pi*acc_multiplier, pi*acc_multiplier, pi*acc_multiplier] # [m, rad, rad, rad]

''''''
# Set the end effector offset no offset in this case
xt = 0.0
yt = 0.0
zt = 0.0
rxt = 0
ryt = 0
rzt = 0


''' INPUTS '''
# Initial position/orientation
x0 = 0.3
y0 = 0.05
z0 = 0.1
rx0 = 0
ry0 = 0
rz0 =0

# Goal position/orientation
xt = 0.25
yt = 0.1
zt = 0.05
rxt = 0
ryt = 0
rzt =0


T = 5 # Trajectory duration
t_cycle = 0.01 # time steps
''''''

pi_res = input('Starting position (m) and orientation (rad) in format: x y z rx ry rz [0.3 0.05 0.1 0 0 0]: ')
pf_res = input('End position (m) and orientation (rad) in format: x y z rx ry rz [0.25 0.1 0.05 0 0 0]: ')
T_res = input('Time duration (seconds): [5]')
print('*'*40)
if pi_res != '':
    x0, y0, z0, rx0, ry0, rz0 = np.fromstring(pi_res, dtype=float, sep=' ')
if pf_res != '':
    xt, yt, zt, rxt, ryt, rzt = np.fromstring(pf_res, dtype=float, sep=' ')
if T_res !='':
    T = float(T_res)



'''
CALCULATION STARTS HERE
'''
# Get the FK and IK classes
fk = m1_fk()
ik = m1_ik()
fk.set_ee(xt, yt, zt, rxt, ryt, rzt)
ik.set_ee(xt, yt, zt, rxt, ryt, rzt)

'''
Get the initial and goal joint positions:
'''
q_init, _ =  ik.closed_ik(x0, y0, z0, rx0, ry0, rz0)
q_goal, _ =  ik.closed_ik(xt, yt, zt, rxt, ryt, rzt)

profiles = []
joint_index = 1
for p_i, p_f, V, A in zip(q_init, q_goal, qdot_max, qddot_max):
    print('Generating trajectory for joint {} from {} to {}'.format(joint_index, p_i, p_f))
    profile = timing_law(p_i, p_f, t_cycle, T, V, V, A, name='Joint ' + str(joint_index)) # Create a timing law per joint to get them in sync
#    profile.generate_trapezoidal_profile() # Generate the timing law with trapezoidal profile
    profile.generate_st_profile()
    profiles.append(profile)
    plot_pvaj(profile.p, profile.v, profile.a, profile.j, vertical=False, title='Joint {}'.format(joint_index))
    joint_index += 1
    
        


