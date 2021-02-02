# -*- coding: utf-8 -*-
"""
Created on Tue Aug 18 08:47:43 2020

@author: Eric
"""
import numpy as np

def latching_filter(SS, LF_C=1, velIdxs=np.zeros((7,1))):
    # velIdxs = velIdxs.astype(int)
    change = SS['xhat'] - SS['xhat_prev']
    change = LF_C * np.square(change)
    change[change>1] = 1
    SS['xhat'] = SS['xhat_prev'] + change * (SS['xhat'] - SS['xhat_prev'])
    # xhat_out[velIdxs] = xhat_raw[velIdxs] # no latching for velocity indices
    SS['xhat'][SS['xhat']>1] = 1
    SS['xhat'][SS['xhat']<-1] = -1
    SS['xhat_prev'] = SS['xhat'].copy()
    
    return SS
