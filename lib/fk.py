# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np

from math import pi, sin, cos, degrees, radians
from tools import homogeneous_transformation, ee_transformation, rotationMatrixToEulerAngles


class m1_fk(object):
    def __init__(self):
        self.l1 = 0.2
        self.l2 = 0.2
        self.set_ee(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.set_dh()
    
    
    '''
    Set end effector offset and rotation
    '''
    def set_ee(self, xt, yt, zt, rxt, ryt, rzt):
        self.xt = xt
        self.yt = yt
        self.zt = zt
        self.rxt = rxt
        self.ryt = ryt
        self.rzt = rzt
        self.ee = [xt, yt, zt, rxt, ryt, rzt]
        self.set_dh()
    

    '''
    Set DH parameters for the Dobot M1
    '''
    
    def set_dh(self):
        self.a = [0, self.l1, self.l2, 0.0]
        self.alpha = [0, 0, 0, 0]
        self.d = [0, 0, 0, 0]
        self.theta = [0, 0, 0, 0]
        

    '''
    Calculate homogeneous transformations for each joint and end effector
    Returns a list with all the transformation matrices
    The last matrix is the end effector transoformation from the base
    '''
    def get_A(self, q):
        self.set_dh()
        A = []
        A.append(homogeneous_transformation(self.alpha[0], self.a[0], 
                                            self.d[0]+q[0], self.theta[0]))
        A.append(homogeneous_transformation(self.alpha[1], self.a[1], 
                                            self.d[1], self.theta[1] + q[1]))
        A.append(homogeneous_transformation(self.alpha[2], self.a[2], 
                                            self.d[2], self.theta[2] + q[2]))
        A.append(homogeneous_transformation(self.alpha[3], self.a[3], 
                                            self.d[3], self.theta[3] + q[3]))
        
        Aee = np.eye(4)
        poses = []
        for An in A:
            Aee = np.matmul(Aee, An)
            poses.append(Aee)

        self.ee_trans = ee_transformation(self.xt, self.yt, self.zt, 0, 0, 0)
        Aee = np.matmul(Aee, self.ee_trans)
        self.ee_trans = ee_transformation(0, 0, 0, self.rxt, self.ryt, self.rzt)
        Aee = np.matmul(Aee, self.ee_trans)
        poses.append(Aee)
        
        return poses
    
    '''
    Get position and orientation of end effector using the homogeneous transformation matrices
    '''
    def fk_using_A(self, q):
        A = self.get_A(q)
        ee = A[-1]
        r = rotationMatrixToEulerAngles(ee[0:3,0:3])
        p = ee[0:3,3]
        
        return np.concatenate((p,r))
        

    '''
    Forward kinematics calculated analytically
    Returns an array with translation and rotation
    '''
    def fk_analytic(self, q):
        xt = self.ee[0]
        yt = self.ee[1]
        zt = self.ee[2]
        rxt = self.ee[3]
        ryt = self.ee[4]
        rzt = self.ee[5]
        px = self.l1*cos(q[1]) + self.l2*cos(q[1]+q[2])  + xt*cos(q[1]+q[2]+q[3]) + yt*sin(q[1]+q[2]+q[3])
        py = self.l1*sin(q[1]) + self.l2*sin(q[1]+q[2])  + xt*sin(q[1]+q[2]+q[3]) + yt*cos(q[1]+q[2]+q[3])
        pz = q[0]+zt
        rx = rxt
        ry = ryt
        rz = q[1] + q[2] + q[3] + rzt
        return np.asarray([px, py, pz, rx, ry, rz])
    
    '''
    Print the position and orientation of all the reference frames in the robot
    '''
    def print_ref_frames(self, poses):
        for i in range(len(poses)):
            if i == len(poses)-1:
                s = 'End effector'
            else:
                s = 'joint {}:'.format(i+1)
            print(s+'\trotation={}\tposition={} '.format(\
                np.round(rotationMatrixToEulerAngles(poses[i][0:3,0:3]),decimals=3),\
                np.round(poses[i][0:3,3],decimals=3)))
    

if __name__=='__main__':
    fk = m1_fk()
    
    '''INPUTS'''
    # joint positions
    q = [0.1, 0, 0, 0]
    # End effector offset/orientation: x, y, z, rx, ry, rz
    xt = 0.01
    yt = 0.01
    zt = 0.05
    rxt = 0
    ryt = 0
    rzt = 0
    ''''''
    
    fk.set_ee(xt, yt, zt, rxt, ryt, rzt)
        
    poses = fk.get_A(q)
    
    print('*'*40)
    fk.print_ref_frames(poses)
    
    print('*'*40)
    print('Solution using homogeneous transformations FK:')
    print(np.round(fk.fk_using_A(q), decimals=3))
    print('Solution using the analytical FK:')
    print(np.round(fk.fk_analytic(q), decimals=3))



    