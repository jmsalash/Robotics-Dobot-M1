#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 14 15:20:25 2019

@author: JoseMa
"""
import numpy as np
from math import pi, sin, cos, degrees, radians, sqrt, atan2


def rot_x(a):
    R = np.asarray([[1, 0, 0],
                    [0, cos(a), -sin(a)],
                    [0, sin(a), cos(a)]], dtype=np.float32)
    return R

def rot_y(a):
    R = np.asarray([[cos(a), 0, sin(a)],
                    [0, 1, 0],
                    [-sin(a), 0, cos(a)]], dtype=np.float32)
    return R

def rot_z(a):
    R = np.asarray([[cos(a), -sin(a), 0],
                    [sin(a), cos(a), 0],
                    [0, 0, 1]], dtype=np.float32)
    return R

def rot_xyz(alpha, beta, gamma):
    R = np.matmul(rot_x(alpha), np.matmul(rot_y(beta),rot_z(gamma)))
    return R


def homogeneous_transformation(alpha, a, d, theta):
    A = np.asarray([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                    [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                    [0, sin(alpha), cos(alpha), d],
                    [0, 0, 0, 1]], dtype=np.float32)
    return A

def ee_transformation(x, y, z, rx, ry, rz):
    A = np.eye(4, dtype=np.float32) 
    Rxyz = rot_xyz(rx, ry, rz)
    A[0:3,0:3] = Rxyz
    A[0:3,3] = [x, y, z]
    return A
    
    

'''
Rotation matrix to Euler angles ZYX
'''
def rotationMatrixToEulerAngles(rmat, rad=True) :
    sy = sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0])
    x = atan2(rmat[2,1] , rmat[2,2])
    y = atan2(-rmat[2,0], sy)
    z = atan2(rmat[1,0], rmat[0,0])
    
    if rad: 
        return np.array([x, y, z])
    else:
        return np.array([degrees(x), degrees(y), degrees(z)])