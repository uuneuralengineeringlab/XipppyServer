import numpy as np
import time

def save_decode_params(SS, RootDir):
    timestr = time.strftime('%Y%m%d-%H%M%S')
    if SS['num_EMG_chans'] == 16:
        SS['decode_fid'] = open(RootDir + r'/decodeparams/decodeKF_' + timestr + r'.kfp', 'wb') # nomad directory
        
    elif SS['num_EMG_chans'] == 32:
        SS['decode_fid'] = open(RootDir + r'/decodeparams32/decodeKF_' + timestr + r'.kfp', 'wb') # nomad directory
    # Write header containg shapes of data to be saved
    header = np.r_[SS['StateMod'].size, SS['sel_feat_idx'].size, SS['K'].size].astype('single')
    # Write header to top of file
    SS['decode_fid'].write(header.astype('single'))    
    SS['decode_fid'].write(np.r_[SS['StateMod'].flatten(), 
                          SS['sel_feat_idx'].flatten() , 
                          SS['K'].flatten()].astype('single'))
    SS['decode_fid'].close()
    
if __name__ == '__main__':
    # for testing
    import feedbackdecode as fd
    SS = fd.initSS()
    RootDir = r'C:\Users\Administrator\Box\CNI\COB\COB_Python'
    SS['StateMod'] = np.arange(SS['StateMod'].size).reshape(SS['StateMod'].shape)
    SS['sel_feat_idx'] = np.arange(SS['sel_feat_idx'].size).reshape(SS['sel_feat_idx'].shape)
    SS['K'] = np.arange(SS['K'].size).reshape(SS['K'].shape)
    save_decode_params(SS, RootDir)

    
    