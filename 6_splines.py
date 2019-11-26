#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 20 18:25:01 2019

@author: JoseMa
"""

import sys
sys.path.append('./lib')
from fk import m1_fk
from ik import m1_ik
from spline import splines
from lib import plot_tools as plt
import numpy as np
from math import pi, radians

'''
JOINT AND VEL/ACC LIMITS DOBOT M1
'''
qmin = [0, -pi/2, -radians(140), -2*pi]
qmax = [0.25, pi/2, radians(140), 2*pi]
qdot_max = [1, pi, pi, pi] # [m, rad, rad, rad]
acc_multiplier = 10 # No specs found about acceleration
qddot_max = [1*acc_multiplier, pi*acc_multiplier, pi*acc_multiplier, pi*acc_multiplier] # [m, rad, rad, rad]
''''''


# End effector rotation/translation
xt = 0.0
yt = 0.0
zt = 0.0
rxt = 0
ryt = 0
rzt = 0
ik = m1_ik()
fk = m1_fk()
#fk.set_ee(xt, yt, zt, rxt, ryt, rzt)
ik.set_ee(xt, yt, zt, rxt, ryt, rzt)
fk.set_ee(xt, yt, zt, rxt, ryt, rzt)
    

T = 14 # Trajectory duration - it will default to fastest possible trajectory
t_cycle = 0.01 # time steps
Vmax = 1 # 1 m/s as in specs for the Z axis
Amax = 10 # No info provided, making it V*10

print('Enter desired passing points in the format: x y')
print('Coordinates should be in meters, and between -0.05 and 0.05 [0.1x0.1m2 area 0,0 in the middle]')
print('Enter \'q\' when done')
print('Default points: (-0.09, -0.05), (-0.05, 0.02), (0.01, -0.01), (0.09, 0.06)')
counter = 1
points = []
while(True):
    p_res = input('Point {}: '.format(counter))
    if p_res in ['q', '']:
        if counter == 1:
            break
        elif counter == 3:
            print('You need at least 4 points end to end')
    else:
        x,y = np.fromstring(p_res, dtype=float, sep=' ')
        points.append((x,y))
    counter += 1

if len(points) == 0:
    points = [(-0.04, -0.02), (-0.02, 0.01), (0.005, -0.005), (0.045, 0.03)]

points = np.asarray(points)

print('Passing points: {}'.format(points))
vi = 0
vf = 0

x0 = points[0][0]
y0 = points[0][1]
z0 = 0.0
rx0 = 0.0
ry0 = 0.0
rz0 = 0.0

xt = points[-1][0]
yt = points[-1][1]
zt = 0
rzt = 0.0
rxt = 0.0
ryt = 0.0
rzt = 0.0

q_i = []
q_f = []



# Time between points - let's make it proportional based on norm distance
L = [0]
spline_duration = []
for i in range(len(points)-1):
    L.append(np.linalg.norm(points[i+1] - points[i]))

L = np.asarray(L)

total_norm = 0 
for l in L:
    total_norm += l

spline_duration = L*T/total_norm
t = [0]
for d in spline_duration:
    t.append(t[-1] + d)
t = t[1:]





# Work in the joint space
# Get joint states for each passing point 
joint_points = {}
for i in range(len(points)-1):
    x0, y0 = points[i]
    xt, yt = points[i+1]
    q_i = ik.closed_ik(x0, y0, z0, rx0, ry0, rz0)[0]
    q_f = ik.closed_ik(xt, yt, zt, rxt, ryt, rzt)[0]
    
    if i == 0:
        for j in range(len(q_i)):
            joint_points[j] = []
#            joint_points[j].append((q_i[j], q_i[j]))
            joint_points[j].append((t[i], q_i[j]))
    for j in range(len(q_i)):
        joint_points[j].append((t[i+1], q_f[j]))
        

# Create splines for each joint
joint_trajectories = []
x = []
y = []
ydot = []
yddot = []
s_list = []
# Go through passing joint states and create a spline for each joint
# If max velocity is violated, 
for key, value in joint_points.items():
    print('Spline for joint {} with states: \n{}'.format(key, np.round(value, decimals=3)))
    value = np.asarray(value)
    vmax = qmax[key]
    s = splines(vi, vf, value, T, t_cycle, vmax)
    s_list.append(s)
    vmax_spline = np.max(np.abs(s.ydot))
    x.append(s.x)
    y.append(s.y)
    ydot.append(s.ydot)
    yddot.append(s.yddot)
#    plt.plot_pvaj(s.y, s.ydot, s.yddot, vertical=False, title='JOINT {}'.format(key+1))
    joint_trajectories.append(s.y)

ydot = np.asarray(ydot)
x = np.asarray(x)
y = np.asarray(y)
yddot = np.asarray(yddot)

for i in range(ydot.shape[0]):
    if np.max(np.abs(ydot[i])) > np.asarray(qmax[i]):
        print('Limiting velocity for joint {}'.format(i+1))        
        # Clip joint velocities to min/max
        ydot[i] = np.clip(ydot[i], qmin[i], qmax[i])
        # Get new acceleration
        yddot[i] = np.gradient(ydot[i])
        tt = ydot.shape[1]
        # Integrate to get the new actual trajectory
        for j in range(1, ydot.shape[1], 1):
            y[i,j] = y[i,j-1] + ydot[i,j]*(t_cycle) + yddot[i,j]*(t_cycle**2)



plt.plot_joints(x, y, ydot, yddot)


joint_trajectories = np.asarray(joint_trajectories).T
p = []
# Do FK on the spline joint states to check cartesian space result
for q in joint_trajectories: #rows = joint positions for each point
    p.append(fk.fk_analytic(q))
    
p = np.asarray(p)
v = np.gradient(p, axis=0)
a = np.gradient(v, axis=0)

plt.plot_cartesian_time(x, p.T, v.T, a.T)

# Draw the trajectory in cartesian space
plt.plot_xy(p[:,0:2], title = 'Trajectory in XY plane')

    

    
   
    
    