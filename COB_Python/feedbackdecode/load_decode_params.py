# -*- coding: utf-8 -*-
import glob
import numpy as np
import os

def load_decode_params(SS, RootDir):
    if SS['num_EMG_chans'] == 16:
        list_of_files = glob.glob(RootDir + r'/decodeparams/*.kfp')
        if list_of_files:
            latest_file = max(list_of_files, key=os.path.getctime)
        else:
            print('No 16 channel decodes (.kfp) available to load')
            return SS
    elif SS['num_EMG_chans'] == 32:
        list_of_files = glob.glob(RootDir + r'/decodeparams32/*.kfp')
        if list_of_files:
            latest_file = max(list_of_files, key=os.path.getctime)
        else:
            print('No 32 channel decodes (.kfp) available to load')
            return SS

    
    # load most recent training file
    decode_fid = open(latest_file, 'br', buffering=0)
    params_contents = np.fromfile(decode_fid, dtype='single')
    decode_fid.close()
    
    # parse file
    header = params_contents[:3].astype('int')
    params_data = params_contents[3:]
    params_data = params_data.reshape(-1,sum(header))
    idxs = np.cumsum(header)
    SS['StateMod'] = params_data[:,:idxs[0]]
    SS['sel_feat_idx'] = params_data[:,idxs[0]:idxs[1]]
    SS['K'] = params_data[:,idxs[1]:idxs[2]]
    SS['StateMod'] = SS['StateMod'].reshape((7,7))
    SS['sel_feat_idx'] = SS['sel_feat_idx'].flatten().astype('int')
    SS['K'] = SS['K'].reshape((7,SS['sel_feat_idx'].size))
    
    return SS

if __name__ == '__main__':
    import feedbackdecode as fd
    SS = fd.initSS()
    RootDir = r'C:\Users\Administrator\Box\CNI\COB\COB_Python'
    SS = load_decode_params(SS, RootDir)
    