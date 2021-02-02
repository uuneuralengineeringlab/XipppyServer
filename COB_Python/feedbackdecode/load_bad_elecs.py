# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 09:03:25 2021

@author: Administrator
"""
import glob
import os
import numpy as np

def load_bad_elecs(SS, RootDir):
    list_of_files = glob.glob(RootDir + r'/bad_elecs/*.elec')
    if list_of_files:
        latest_file = max(list_of_files, key=os.path.getctime)
    else:
        print('No bad electrode files (.elec) available to load')
        return SS

    
    # load most recent bad electrode file
    bad_elec_FID = open(latest_file, 'br', buffering=0)
    params_contents = np.fromfile(bad_elec_FID, dtype='single')
    bad_elec_FID.close()
    
    # parse file
    SS['bad_EMG_elecs'] = params_contents[1:].astype('int')
    
    return SS
