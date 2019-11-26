#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 20 19:40:04 2019

@author: JoseMa
"""
from matplotlib import pyplot as plt

def plot_pvaj(p=None, v=None, a=None, j=None, vertical=True, title = ''):
    if vertical:
        f = plt.figure(figsize=(5,15))
        ax1 = f.add_subplot(411)
        ax2 = f.add_subplot(412)
        ax3 = f.add_subplot(413)
        ax4 = f.add_subplot(414)
    else:
        f = plt.figure(figsize=(20,5))
        ax1 = f.add_subplot(141)
        ax2 = f.add_subplot(142)
        ax3 = f.add_subplot(143)
        ax4 = f.add_subplot(144)
    
    if p is not None:
        ax1.plot(p, label='P')
        ax1.set(ylabel='rad', title = 'Position')
        ax1.grid(True)
    
    if v is not None:
        ax2.plot(v, label='V')
        ax2.set(ylabel='rad/s', title = 'Velocity')
        ax2.grid(True)
    
    if a is not None:
        ax3.plot(a, label='A')
        ax3.set(ylabel='rad/s2', title = 'Acceleration')
        ax3.grid(True)
    
    if j is not None:
        ax4.plot(j, label='J')
        ax4.set(xlabel='steps', ylabel='rad/s3', title = 'Jerk')
        ax4.grid(True)
    
    f.tight_layout()
    f.suptitle(title, fontsize=18)
    f.subplots_adjust(top=0.88)
    
    plt.show()

    
def plot_cartesian(p=None, v=None, a=None, title='TRANSLATION AND ROTATION POSITION, VELOCITY AND ACCELERATION'):
    f = plt.figure(figsize=(15,15))
            
    ax1 = f.add_subplot(321)
    ax2 = f.add_subplot(322)
    ax3 = f.add_subplot(323)
    ax4 = f.add_subplot(324)
    ax5 = f.add_subplot(325)
    ax6 = f.add_subplot(326)
    
    
    if p is not None:
        ax1.plot(p[:,0], label='px')
        ax1.plot(p[:,1], label='py')
        ax1.plot(p[:,2], label='pz')
        ax1.set(ylabel='m', title = 'Translation')
        ax1.legend()
        ax1.grid(True)
    
        ax2.plot(p[:,3], label='rx')
        ax2.plot(p[:,4], label='ry')
        ax2.plot(p[:,5], label='rz')
        ax2.set(ylabel='rad', title = 'Rotation')
        ax2.legend()
        ax2.grid(True)
    
    if v is not None:
        ax3.plot(v[:,0], label='vx')
        ax3.plot(v[:,1], label='vy')
        ax3.plot(v[:,2], label='vz')
        ax3.set(ylabel='m/s', title = 'Linear Velocity')
        ax3.legend()
        ax3.grid(True)
        
        
        ax4.plot(v[:,3], label='wx')
        ax4.plot(v[:,4], label='wy')
        ax4.plot(v[:,5], label='wz')
        ax4.set(ylabel='rad/s', title = 'Angular Velocity')
        ax4.legend()
        ax4.grid(True)
    
    if a is not None:
        ax5.plot(a[:,0], label='ax')
        ax5.plot(a[:,1], label='ay')
        ax5.plot(a[:,2], label='az')
        ax5.set(xlabel='step', ylabel='m/s2', title = 'Linear Acc')
        ax5.legend()
        ax5.grid(True)
        
        
        ax6.plot(a[:,3], label='wdotx')
        ax6.plot(a[:,4], label='wdoty')
        ax6.plot(a[:,5], label='wdotz')
        ax6.set(xlabel='step', ylabel='rad/s2', title = 'Angular Acc')
        ax6.legend()
        ax6.grid(True)
    
    
    f.tight_layout()
    f.suptitle(title, fontsize=18)
    f.subplots_adjust(top=0.93)
    
    plt.show()

def plot_cartesian_time(x, p=None, v=None, a=None, title='TRANSLATION AND ROTATION POSITION, VELOCITY AND ACCELERATION'):
    f = plt.figure(figsize=(15,15))
            
    ax1 = f.add_subplot(321)
    ax2 = f.add_subplot(322)
    ax3 = f.add_subplot(323)
    ax4 = f.add_subplot(324)
    ax5 = f.add_subplot(325)
    ax6 = f.add_subplot(326)
    
    
    if p is not None:
        ax1.plot(x[0], p[0], label='px')
        ax1.plot(x[0], p[1], label='py')
        ax1.plot(x[0], p[2], label='pz')
        ax1.set(ylabel='m', title = 'Translation')
        ax1.legend()
        ax1.grid(True)
    
        ax2.plot(x[0], p[3], label='rx')
        ax2.plot(x[0], p[4], label='ry')
        ax2.plot(x[0], p[5], label='rz')
        ax2.set(ylabel='rad', title = 'Rotation')
        ax2.legend()
        ax2.grid(True)
    
    if v is not None:
        ax3.plot(x[0], v[0], label='vx')
        ax3.plot(x[0], v[1], label='vy')
        ax3.plot(x[0], v[2], label='vz')
        ax3.set(ylabel='m/s', title = 'Linear Velocity')
        ax3.legend()
        ax3.grid(True)
        
        
        ax4.plot(x[0], v[3], label='wx')
        ax4.plot(x[0], v[4], label='wy')
        ax4.plot(x[0], v[5], label='wz')
        ax4.set(ylabel='rad/s', title = 'Angular Velocity')
        ax4.legend()
        ax4.grid(True)
    
    if a is not None:
        ax5.plot(x[0], a[0], label='ax')
        ax5.plot(x[0], a[1], label='ay')
        ax5.plot(x[0], a[2], label='az')
        ax5.set(xlabel='step', ylabel='m/s2', title = 'Linear Acc')
        ax5.legend()
        ax5.grid(True)
        
        
        ax6.plot(x[0], a[3], label='wdotx')
        ax6.plot(x[0], a[4], label='wdoty')
        ax6.plot(x[0], a[5], label='wdotz')
        ax6.set(xlabel='step', ylabel='rad/s2', title = 'Angular Acc')
        ax6.legend()
        ax6.grid(True)
    
    
    f.tight_layout()
    f.suptitle(title, fontsize=18)
    f.subplots_adjust(top=0.93)
    
    plt.show()
    
def plot_joints(x, p, v, a, vertical=False):
    
    n_joints = p.shape[0]
    f = plt.figure(figsize=(5,15))
    if vertical:
        f = plt.figure(figsize=(5,15))
        ax1 = f.add_subplot(411)
        ax2 = f.add_subplot(412)
        ax3 = f.add_subplot(413)
    else:
        f = plt.figure(figsize=(20,5))
        ax1 = f.add_subplot(141)
        ax2 = f.add_subplot(142)
        ax3 = f.add_subplot(143)
    
    for i in range(n_joints):
        ax1.plot(x[i], p[i], label='q{}'.format(i+1))
        ax2.plot(x[i], v[i], label='qdot{}'.format(i+1))
        ax3.plot(x[i], a[i], label='qddot{}'.format(i+1))
        ax1.set(ylabel='rad', title = 'Position')
        ax2.set(ylabel='rad/s', title = 'Velocity')
        ax3.set(ylabel='rad/s2', title = 'Acceleration')
    
    ax1.legend()
    ax1.grid(True)
    ax2.legend()
    ax2.grid(True)
    ax3.legend()
    ax3.grid(True)

    
    f.tight_layout()
    f.suptitle('JOINTS', fontsize=18)
    f.subplots_adjust(top=0.88)
    
    plt.show()
    

def plot_xy(p, title=''):
    x = p[:,0]
    y = p[:,1]
    plt.plot(x,y)
    plt.title(title)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()
