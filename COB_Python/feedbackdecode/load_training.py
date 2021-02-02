import numpy as np
import glob
import os
import feedbackdecode as fd

def load_training(SS, RootDir):
    # everything below probably should be parallelized
    
    # load most recent training file
    # import pdb; pdb.set_trace()
    # list_of_files = glob.glob('/srv/data/training_KDFs/*.kdf')
    list_of_files = glob.glob(RootDir + r'/training_KDFs/*.kdf')
    latest_file = max(list_of_files, key=os.path.getctime)
    # print(f'Training on {latest_file}')
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
    
    # train KF then get steady state
    (A, W, _, _, _, H, Q, _) = fd.kf_train_cob(kinematics, features)
    print('Kalman Trained')
    
    SS['K'], SS['StateMod'], _ = fd.kf_getss_cob(A, W, H, Q)
    print('SS Kalman Trained')
    
    SS['kf_load_train'] = None
    return SS['K'], SS['StateMod'], SS['kf_load_train']
