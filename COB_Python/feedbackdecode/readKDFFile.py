# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 12:25:13 2020

@author: TNT
"""
import glob
import os
import numpy as np
def readKDFFile(SS, RootDir):

    # load most recent training file
    if SS['numChansEMG'] == 16:
        list_of_files = glob.glob(RootDir + r'/training_KDFs/*.kdf')
    elif SS['numChansEMG'] == 32:
        list_of_files = glob.glob(RootDir + r'/training_KDFs32/*.kdf')
    else:
        raise ValueError
    latest_file = max(list_of_files, key=os.path.getctime)
    SS['train_fid'] = open(latest_file, 'br', buffering=0)
    train_contents = np.fromfile(SS['train_fid'], dtype='single')
    SS['train_fid'].close()
    
    # parse file
    header = train_contents[:3].astype('int')
    train_data = train_contents[3:]
    train_data = train_data.reshape(-1,sum(header))
    idxs = np.cumsum(header)
    NIP_times = train_data[:,:idxs[0]]
    features = train_data[:,idxs[0]:idxs[1]]
    kinematics = train_data[:,idxs[1]:idxs[2]]
    
     
    
    return SS, kinematics, features