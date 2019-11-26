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
from plot_tools import plot_cartesian
import numpy as np

from math import radians, pi

'''
JOINT AND VEL/ACC LIMITS DOBOT M1
'''
'''
qmin = [0, -pi/2, -radians(140), -2*pi]
qmax = [0.25, pi/2, radians(140), 2*pi]
qdot_max = [1, pi, pi, pi] # [m, rad, rad, rad]
acc_multiplier = 10 # No specs found about acceleration
qddot_max = [1*acc_multiplier, pi*acc_multiplier, pi*acc_multiplier, pi*acc_multiplier] # [m, rad, rad, rad]
'''

'''
END EFFECTOR OFFSETS AND ROTATIONS
'''
# Set the end effector offset - no offset in this case
xt = 0.0
yt = 0.0
zt = 0.0
rxt = 0
ryt = 0
rzt = 0

''''''


''' INPUTS '''
# Initial position/orientation
x0 = 0.3
y0 = 0.05
z0 = 0.1
rx0 = 0
ry0 = 0
rz0 = pi/2

# Goal position/orientation
xt = 0.25
yt = 0.1
zt = 0.05
rxt = 0
ryt = 0
rzt = -pi/2


T = 5 # Trajectory duration - it will default to fastest possible trajectory
t_cycle = 0.01 # time steps
Vmax = 1 # 1 m/s as in specs for the Z axis
Amax = 10 # No info provided, making it V*10
''''''




'''
CALCULATION STARTS HERE
'''
# Get the FK and IK classes
fk = m1_fk()
ik = m1_ik()
fk.set_ee(xt, yt, zt, rxt, ryt, rzt)
ik.set_ee(xt, yt, zt, rxt, ryt, rzt)

pi_res = input('Starting position (m) and orientation (rad) in format: x y z rx ry rz [0.3 0.05 0.1 0 0 1.57]: ')
pf_res = input('End position (m) and orientation (rad) in format: x y z rx ry rz [0.25 0.1 0.05 0 0 -1.57]: ')
T_res = input('Time duration (seconds): [5]')
print('*'*40)
if pi_res != '':
    p_i = np.fromstring(pi_res, dtype=float, sep=' ')
if pf_res != '':
    p_f = np.fromstring(pf_res, dtype=float, sep=' ')
if T_res !='':
    T = float(T_res)


p_i = np.asarray([x0, y0, z0, rx0, ry0, rz0])
p_f = np.asarray([xt, yt, zt, rxt, ryt, rzt])
p_diff = p_f[0:3] - p_i[0:3]
L = np.linalg.norm(p_diff) # Length of the path from point B to point A

p = None
v = None
a = None
profiles = []
for p0, pf in zip(p_i, p_f):
    profile = timing_law(p0, pf, t_cycle, T, Vmax, Vmax, Amax)
    profile.generate_st_profile()
    profiles.append(profile)
    if p is None:
        p = profile.p.reshape(profile.p.shape[0],1)
        v = profile.v.reshape(profile.p.shape[0],1)
        a = profile.a.reshape(profile.p.shape[0],1)
    else:
        p = np.hstack((p,profile.p.reshape(profile.p.shape[0],1)))
        v = np.hstack((v,profile.v.reshape(profile.p.shape[0],1)))
        a = np.hstack((a,profile.a.reshape(profile.p.shape[0],1)))
    
    
plot_cartesian(p, v, a)
    
#profile.plot_graphs_vertical()

