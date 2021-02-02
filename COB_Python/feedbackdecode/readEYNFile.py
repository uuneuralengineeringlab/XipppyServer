# -*- coding: utf-8 -*-
"""
Created on Tue Nov  3 10:38:49 2020

@author: Administrator
"""

import numpy as np
def readEYNFile( filePath):

    # load most recent training file

    EYN_fid = open(filePath, 'br', buffering=0)
    train_contents = np.fromfile(EYN_fid, dtype='single')
    EYN_fid.close()
    
    # parse file
    header = train_contents[:4].astype('int')
    train_data = train_contents[4:]
    train_data = train_data.reshape(-1,sum(header))
    idxs = np.cumsum(header)
    NIP_times = train_data[:,:idxs[0]]
    features = train_data[:,idxs[0]:idxs[1]]
    xhat = train_data[:,idxs[1]:idxs[2]]
    curSensors = train_data[:,idxs[2]:idxs[3]]
    
     
    
    return  NIP_times, features, xhat, curSensors