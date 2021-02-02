# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 16:24:18 2020

@author: Eric
"""
import numpy as np

def decode_threshold(SS):
    restPos = np.zeros(SS['xhat'].shape)
    # ### temporary for debugging
    # SS['xhat'][0] = 0.05
    # SS['xhat'][1] = 0.5
    # SS['xhat'][2] = -0.05
    # SS['xhat'][3] = -0.5
    # SS['threshold'] = 0.1
    # ### end temporary
    pos = (SS['xhat']>=restPos) # check if xhat is greater or less than the restPos
    if any(pos):
        SS['xhat'][pos] = (SS['xhat'][pos]-SS['threshold'])/(1-SS['threshold']) #apply flexion gains/thresholds
    if any(~pos):
        SS['xhat'][~pos] = (SS['xhat'][~pos]+SS['threshold'])/(1-SS['threshold']) #apply extension gains/thresholds
    dynidx = np.logical_or(np.logical_and(SS['xhat']>restPos, ~pos), np.logical_and(SS['xhat']<restPos, pos))
    SS['xhat'][dynidx] = restPos[dynidx]
    return SS
