# -*- coding: utf-8 -*-
"""
Created on Tue Dec  1 12:04:30 2020

@author: Administrator
"""

import numpy as np

    
    
def norm_pos_digit(self, sensor, ind):
        # normalize [-1, 1]
        rest = self.rest_pos_sens[ind]
        minpos = self.min_pos_sens[ind]
        maxpos = self.max_pos_sens[ind]
        if sensor >= rest:
            mapped_sensor = (sensor-rest)/abs(maxpos-rest) 
        else:
            mapped_sensor = (sensor-rest)/abs(minpos-rest) 
        
        return np.clip(mapped_sensor,-1,1)
    
    
def norm_pos_wrist(self, sensor, limits): 
        # normalize [-1, 1]
        rest = limits[0]
        negmax = limits[1]
        posmax = limits[2]
        if sensor >= rest:
            mapped_sensor = sensor/posmax 
        else:
            mapped_sensor = sensor/negmax
        
        return np.clip(mapped_sensor,-1,1)
    
    
min_pos_aci  = np.array([  0,    0,   0,  500, 1024, 1024], dtype=np.int16)
rest_pos_aci = np.array([225,  400, 350,  900,    0,    0], dtype=np.int16) 
max_pos_aci  = np.array([450, 1024, 950, 1024, 1024, 1024], dtype=np.int16)

min_pos_sens  = np.array([   0,    0,    0,  3000, 3520,  7680], dtype=np.int16)
max_pos_sens  = np.array([2450, 5575, 5330,  6360, 3520, 11200], dtype=np.int16)
with np.errstate(divide='ignore', invalid='ignore'):
    rest_pos_sens = (rest_pos_aci-min_pos_aci) / (max_pos_aci-min_pos_aci) * (max_pos_sens-min_pos_sens) + min_pos_sens
    rest_pos_sens[4:6] = 0
    
    

#thumb: 0->5760
#ind: 0->5760
#mrp: 0->5760
#thumbint: 0->4800
#wristfe: -3520->3520
#wristrot: -7680->11200   
norm_pos_digit(5760,0)