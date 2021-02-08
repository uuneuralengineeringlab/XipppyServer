# -*- coding: utf-8 -*-

import glob
import numpy as np
import os

def load_stim_params(SS, RootDir):
    # find most recent file
    list_of_files = glob.glob(RootDir + r'/stimparams/*.sp')
    if list_of_files:
        latest_file = max(list_of_files, key=os.path.getctime)
    else:
        print('No stim params (.sp) available to load')
        return SS

    # load
    stimparamFID = open(latest_file, 'br', buffering=0)
    params_contents = np.fromfile(stimparamFID, dtype='single')
    stimparamFID.close()
    
    # parse
    header = params_contents[:2].astype('int')
    params_data = params_contents[2:]
    SS['stim_params'] = params_data.reshape(header)
    
    return SS
    