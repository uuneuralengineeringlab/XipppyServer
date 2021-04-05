# -*- coding: utf-8 -*-
"""
Created on Fri Apr  2 14:08:42 2021

@author: Administrator
"""
import numpy as np
import serial # pySerial
ard = serial.Serial('/dev/ttyUSB2')
ard.baudrate = 250000

msg = np.array([255, 0, 0, 0, 0, 0],dtype=np.uint8)
ard.write(msg)