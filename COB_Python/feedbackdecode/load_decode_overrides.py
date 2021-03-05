# -*- coding: utf-8 -*-
import glob
import numpy as np
import os

def load_decode_overrides(SS, RootDir):
    list_of_files = glob.glob(RootDir + r'/decode_overrides/*.dor')
    if list_of_files:
        latest_file = max(list_of_files, key=os.path.getctime)
    else:
        print('No decode overrides available to load')
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
    SS['kin'] = params_data[:,:idxs[0]].flatten().astype(np.single)
    SS['lock_DOF'] = params_data[:,idxs[0]:idxs[1]].flatten().astype('bool')
    SS['mirror_DOF'] = params_data[:,idxs[1]:idxs[2]].flatten().astype('int')
    # SS['kin'] = SS['kin'].reshape((7,7))
    # SS['lock_DOF'] = SS['lock_DOF'].flatten().astype('int')
    # SS['mirror_DOF'] = SS['mirror_DOF'].reshape((7,SS['sel_feat_idx'].size))
    
    return SS

if __name__ == '__main__':
    import feedbackdecode as fd
    SS = fd.initSS()
    RootDir = r'C:\Users\Administrator\Box\CNI\COB\COB_Python'
    SS = load_decode_params(SS, RootDir)
    