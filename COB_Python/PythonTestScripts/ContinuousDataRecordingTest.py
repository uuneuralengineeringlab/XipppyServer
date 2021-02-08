# -*- coding: utf-8 -*-
"""
Created on Tue Jan  5 08:36:21 2021

@author: Administrator
"""

import numpy as np
import time
import xipppy as xp
import feedbackdecode as fd


## make some random but realistic timestamps (old)
times = np.arange(1,11)*990 + np.ceil((np.random.rand(10)-0.5) * 4) * 30
# 990 ms expected NIP cycles per loop +- 120 NIP cycles (120 NIP = 4 ms)
# NIP times are always (apparently) multiples of 30 (1 ms)

diffs = np.diff(times)

diffs - 990


## get some real time stamps similar to the loop
SS = fd.initSS()
xp._open()

NIPtimes = []
d = []
for i in range(10):
    NIPtimes.append(xp.time())
    temp = xp.cont_lfp(37, SS['all_EMG_chans'], 0)
    d.append(np.reshape(temp[0],(SS['num_EMG_chans'],37)))
    time.sleep(33/1000)
    
NIPtimes = np.array(NIPtimes)

np.diff(NIPtimes)


# d = xp.cont_lfp(37, SS['all_EMG_chans'], 0)
# d = np.reshape(d[0],(SS['num_EMG_chans'],37)) # want time x channels


## figure out how to piece them together....
NIPtimes0 = NIPtimes - NIPtimes[0]
NIPtimes0ms = NIPtimes0/30
NIPtimes0msdiff = np.diff(NIPtimes0ms)

prevTime = 0
cont = np.empty((32,0))
for i in range(len(NIPtimes)):
    curTime = NIPtimes0[i]
    elapsedTime = int((curTime - prevTime)/30)
    prevTime = curTime
    
    dfeat = d[i]
    newdata = dfeat[:,37-np.clip(elapsedTime,0,37):]
    cont = np.hstack((cont,newdata))
    
    