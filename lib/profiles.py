# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
from matplotlib import pyplot as plt
from plot_tools import plot_pvaj

from math import pi, sqrt, sin

'''
Acceleration calculation with integration approximation
'''
def acceleration_integrator(a_cur, j_cur, ts):
        a_next = a_cur + j_cur*ts
        return a_next

'''
Velocity calculation with integration approximation
'''
    
def velocity_integrator(v_cur, a_cur, j_cur, ts):
    v_next = v_cur + a_cur*ts + j_cur*(ts**2)
    return v_next

'''
Position calculation with integration approximation
'''

def position_integrator(pos_cur, v_cur, a_cur, j_cur, ts):
    pos_next = pos_cur + v_cur*ts + a_cur*(ts**2) + j_cur*(ts**3)
    return pos_next

class timing_law(object):
    '''
    p_i: initial position
    p_f: final position
    t_cycle: cycle time in seconds
    T: timing law duration in seconds. If T is lower than minimum feasible value, it will return the fastest possible timing law
    V = Max velocity
    A = Max acceleration
    Jerk is hardcoded to A*1.33
    '''
    def __init__(self, p_i, p_f, t_cycle, T=None, V=None, Vmax=None, A=None, name = ''):
        self.p_i = p_i
        self.p_f = p_f
        self.P = abs(p_f - p_i)
        # Fake Vmax. I realised right at the end that Vmax was actually an input
        # I designed the profiles to be as fast and smooth as possible
        # It will never go above desired V.
aa        # If T > minimum required time, the trajectory is scaled accordingly
        if Vmax is None:
            self.V = V
        else:
            self.V = Vmax
        self.A = A
        self.T = T
        self.J = A*1.33
        self.t_cycle = t_cycle
        self.name = name

        self.j = []
        self.a = []
        self.v = []
        self.p = []
        self.pi = 0
        
        # Sign of the movement
        if type(self.p_i) == np.ndarray:
            self.m = []
            for i in range(self.p_i.shape[0]):
                if self.p_i[i] > self.p_f[i]:
                    self.m.append(-1)
                else:
                    self.m.append(1)
        else:
            if self.p_i > self.p_f:
                self.m = -1
            else:
                self.m=1
        

    '''
    Generate a normalized timing law with a trapezoidal profile
    This can be used in both joint and cartesian space
    '''
    def generate_normalised_trapezoidal_profile(self):
        self.L =np.linalg.norm(self.p_f - self.p_i)
        self.sigma = []
        self.scale_trapezoidal()
        
        for tc in np.arange(0, self.Tmin+self.t_cycle, self.t_cycle, dtype=np.float64):
            self.sigma.append(self._trapezoidal(tc))
            
        self.sigma = np.asarray(self.sigma)
        self.s = self.sigma/self.L
        
        self.s_newdim = []
        self.p_i_newdim = []
        if self.p_i.shape[0] > 1:
            for i in range(self.p_i.shape[0]):
                self.s_newdim.append(self.s)
            for i in range(self.s.shape[0]):
                self.p_i_newdim.append(self.p_i)
            self.p_i_newdim = np.asarray(self.p_i_newdim)
            self.s_newdim = np.asarray(self.s_newdim).T
            self.p_i = self.p_i.reshape((self.p_i.shape[0],1))
            self.p_f = self.p_f.reshape((self.p_f.shape[0],1))
            
        else:
            self.s_newdim = self.s
        
        self.p = []
        self.v = []
        self.a = []
        self.j = []
        self.sdot = np.gradient(self.s)
        self.sddot =  np.gradient(self.sdot)
        self.sdddot =  np.gradient(self.sddot)
        for s, sdot, sddot, sdddot in zip(self.s, self.sdot, self.sddot, self.sdddot):
            self.p.append(self.p_i + s*(self.p_f - self.p_i))
            self.v.append(sdot*(self.p_f - self.p_i))
            self.a.append(sddot*(self.p_f - self.p_i))
            self.j.append(sdddot*(self.p_f - self.p_i))
        
        self.p = np.asarray(self.p)
        self.v = np.asarray(self.v)
        self.a = np.asarray(self.a)
        self.j = np.asarray(self.j)
    
    
    '''
    Generate a timing law with a trapezoidal profile
    '''
    def generate_trapezoidal_profile(self):
        self.L =np.linalg.norm(self.p_f - self.p_i)
        self.sigma = []
        
        self.scale_trapezoidal()
        

        for tc in np.arange(0, self.Tmin, self.t_cycle, dtype=np.float64):
            self.p.append(self.p_i+self.m*self._trapezoidal(tc))
            
        self.p = np.asarray(self.p)
        self.v = np.gradient(self.p)
        self.a = np.gradient(self.v)
        self.j = np.gradient(self.a)


    '''
    Scale the trapezoidal timing law
    '''
    def scale_trapezoidal(self):
        self.Ts = self.V/self.A # Cruising time
        self.Tmin = (self.L*self.A + self.V**2)/(self.A*self.V) # Total time
        if self.Tmin < self.T:
            self.scaling = self.T/self.Tmin
            print('Scaling to T={}'.format(self.T))
        else:
            print('Unable to meet timing requirement - performing fastest trajectory')
            self.scaling = 1
            
        self.V = self.V/self.scaling
        self.A = self.A/(self.scaling**2)
        self.Ts = self.V/self.A # Cruising time
        self.Tmin = (self.L*self.A + self.V**2)/(self.A*self.V) # Total time
        print('New min time: {}'.format(self.Tmin))    
        
    '''   
    Trapezoidal timing law
    '''
    def _trapezoidal(self, tc):
        if tc <= self.Ts:
            return (self.A*tc**2)/2
        elif tc >= self.Tmin-self.Ts:
            return (-self.A*(tc-self.Tmin)**2)/2 + self.V*self.Tmin -(self.V**2)/self.A
        else:
            return self.V*tc - (self.V**2)/(2*self.A)
    
    def generate_profile(self):
        if type(self.p_i) == np.ndarray:
            org_p_i = self.p_i
            org_p_f = self.p_f
            for p_i, p_f in zip(org_p_i, org_p_f):
                self.p_i = p_i
                self.p_f = p_f
                
    
    '''
    Generate a timing law with a trapezoidal profile
    '''
    def generate_st_profile(self):
        self.Ta = min((self.A*pi)/self.J, sqrt(self.V*2*pi/self.J))
        self.Tv = (self.P*2*pi/(self.J*(self.Ta**2))) - self.Ta
        if self.Tv < 0: # Ta too long, Jerk is not high enough
            print('Not enough acceleration')
            self.Tv = 0 # So it must be self.Ta = self.T/2
            self.Ta = self.T/2
            self.J = 2*pi*self.P/((self.Ta**2)*(self.T-self.Ta))
        self.Tmin = self.Tv + 2*self.Ta
        print('Estimated minimum time: {}'.format(self.Tmin))
        
        if self.Tmin < self.T:
            self.scaling = self.T/self.Tmin
            print('Scaling to T={}'.format(self.T))
            self.V = self.V/self.scaling
            self.A = self.A/(self.scaling**2)
            self.J = self.J/(self.scaling**3)
            
            self.Ta = min((self.A*pi)/self.J, sqrt(self.V*2*pi/self.J))
            self.Tv = (self.P*2*pi/(self.J*(self.Ta**2))) - self.Ta
            self.Tmin = self.Tv + 2*self.Ta
            print('New estimated minimum time: {}'.format(self.Tmin))
        else:
            print('Unable to meet timing requirement - performing fastest trajectory with Tmin={}'.format(self.Tmin))
            self.scaling = 1
        self.T = self.Tmin
            
        self.j = [0]
        self.a = [0]
        self.v = [0]
        self.p = [0]
        for tc in np.arange(0, self.T, self.t_cycle, dtype=np.float64):
            self.j.append(self._jerk(tc))
#            self.j = np.asarray(self.j)
            self.a.append(acceleration_integrator(self.a[-1], self.j[-1], self.t_cycle))
            self.v.append(velocity_integrator(self.v[-1], self.a[-1], self.j[-1], self.t_cycle))
            self.p.append(position_integrator(self.p[-1], self.v[-1], self.a[-1], self.j[-1], self.t_cycle))
        
        self.Tmin = len(self.j)
        self.p = self.p_i + self.m*np.asarray(self.p)
        self.v = np.gradient(self.p)
        self.a = np.gradient(self.v)
        self.j = np.gradient(self.a)
        
        
    '''   
    ST curve timing law
    http://repository.supsi.ch/9584/1/baraldo_valente_icra2017_final.pdf
    With a small change in self.Ta + self.Tv <= tc < 2*self.Ta + self.Tv;
    research paper seems to have a mistake
    '''
    def _jerk(self, tc):
        if tc <= self.Ta:
            return self.J*sin((2*pi)*tc/self.Ta)
        elif self.Ta + self.Tv <= tc < 2*self.Ta + self.Tv:
            return -self.J*sin((2*pi)*(tc-self.Tv)/(self.Ta))
        else:
            return 0
        
    
    '''
    Generate the timing law for an S-curve profile
    '''
    def generate_s_curve_profile(self):
        t1 = self.A/self.J
        t2 = self.V/self.A - self.A/self.J
        t3 = self.P/self.V - self.V/self.A - self.A/self.J
        t4 = t1
        t5 = t2
        self.Tmin=2*t1 + t2 + t3 + 2*t4 + t5

        print('Minimum time: {}'.format(self.Tmin)) 
        if self.Tmin > self.T:
            print('Unable to meet desired time T. Minimum time: {}',format(self.Tmin))
            self.scaling=1
        else:
            # Scale as required
            print('Scaling to T={}'.format(self.T))
            self.scaling = self.T/self.Tmin
        self.V = self.V/self.scaling
        self.A = self.A/(self.scaling**2)
        self.J = self.J/(self.scaling**3)
        
        self.j = [0]
        self.a = [0]
        self.v = [0]
        self.p = [0]
        tc = -self.t_cycle
        while(True):
            tc += self.t_cycle
            self.j.append(self._s_curve(tc))
            self.a.append(acceleration_integrator(self.a[-1], self.j[-1], self.t_cycle))
            self.v.append(velocity_integrator(self.v[-1], self.a[-1], self.j[-1], self.t_cycle))
            self.p.append(position_integrator(self.p[-1], self.v[-1], self.a[-1], self.j[-1], self.t_cycle))
      
#            if tc>=self.T-self.t_cycle:
            if self.p[-1] > self.P:#abs(self.p[-1] - self.P) < 0.0001:
                break
        self.p = self.p_i + self.m*np.asarray(self.p)
        self.v = np.gradient(self.p)
        self.a = np.gradient(self.v)
        self.j = np.gradient(self.a)
        self.Tmin = len(self.p)*t_cycle
        print('New min time: {}'.format(self.Tmin))
  
            
    '''
    S curve jerk calculation
    From: https://wp.kntu.ac.ir/delrobaei/files/NeuromuscularSystems/Tutorials2018/KNTU_NeuroMuscularSys_2018_TimeOptimalMinJerk_MKia.pdf
    '''
    def _s_curve(self, t):
    
        if self.P/self.V < (self.A/self.J + self.V/self.A):
            b = self.A/self.J
            self.V = self.A * (-b + sqrt(b**2 + 4*self.P/self.A)) / 2
          
        if self.V*self.J < self.A**2:
            self.A = sqrt(self.J*self.V)
        
    
        t1 = self.A/self.J
        t2 = self.V/self.A - self.A/self.J
        t3 = self.P/self.V - self.V/self.A - self.A/self.J
        t4 = t1
        t5 = t2
        t = round(t / self.t_cycle) * self.t_cycle
        t1 = round(t1 / self.t_cycle) * self.t_cycle
        t2 = round(t2 / self.t_cycle) * self.t_cycle
        t3 = round(t3 / self.t_cycle) * self.t_cycle
        t4 = round(t4 / self.t_cycle) * self.t_cycle
        t5 = round(t5 / self.t_cycle) * self.t_cycle
        K = 2*self.P/(t1*(t1+t2)*(t5+2*t4+2*t3+2*t1+t2))
        
        
        if t < (t1):
            r = +K
        elif t < (t1 + t2):
            r = 0
        elif t < (2*t1 + t2):
            r = -K
        elif t < (2*t1 + t2 + t3):
            r = 0
        elif t < (2*t1 + t2 + t3 + t4):
            r = -K
        elif t < (2*t1 + t2 + t3 + t4 + t5):
            r = 0
        elif t < (2*t1 + t2 + t3 + 2*t4 + t5):
            r = K
        else:
            r = 0
        return r
    

if __name__ == '__main__':

    '''
    INPUTS
    '''
    t_cycle = 0.001
    p_i = pi # initial position
    p_f = -pi # final position

    V = 1 # Velocity (rad/sec)
    A = 3 # Acceleration (rad/sec2)
    T = 10 # Duration
    timing = timing_law(p_i, p_f, t_cycle, T, V, A)
    
#    timing.generate_trapezoidal_profile()
#    timing.generate_s_curve_profile()
    timing.generate_st_profile()
    
    plot_pvaj(timing.p, timing.v, timing.a, timing.j, vertical=True)

    

    



