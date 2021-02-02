# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 13:09:52 2020

@author: Administrator
"""

import time
import numpy as np
def drummer():
    counter = 0
    # time.sleep(time.time() * 8 % 1 / 8) # enable to sync clock for demo
    times = []
    sleeps = []
    while counter < 30 * 5:
        counter += 1
        times.append(time.time())
        sleeps.append(1/30 - time.time() * 30 % 1 / 30)
        time.sleep(1/30 - time.time() * 30 % 1 / 30)
    times = np.array(times)
    sleeps = np.array(sleeps)
    return times, sleeps

if __name__ == '__main__':
    times, sleeps = drummer()
    
    diffs = np.diff(times)
    meandiff = np.mean(diffs)
    stddiff = np.std(diffs)