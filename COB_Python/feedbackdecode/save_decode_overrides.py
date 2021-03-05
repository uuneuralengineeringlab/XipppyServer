import numpy as np
import time

def save_decode_overrides(SS, RootDir=r'/srv/data'):
    timestr = time.strftime('%Y%m%d-%H%M%S')
    fid = open(RootDir + r'/decode_overrides/decodeOR_' + timestr + r'.dor', 'wb') # nomad directory
        
    # Write header containg shapes of data to be saved
    header = np.r_[SS['kin'].size, SS['lock_DOF'].size, SS['mirror_DOF'].size].astype('single')
    # Write header to top of file
    fid.write(header.astype('single'))    
    fid.write(np.r_[SS['kin'], 
                    SS['lock_DOF'], 
                    SS['mirror_DOF']].astype('single'))
    fid.close()
    
if __name__ == '__main__':
    # for testing
    import feedbackdecode as fd
    SS = fd.initSS()
    RootDir = r'C:\Users\Administrator\Box\CNI\COB\COB_Python'
    SS['StateMod'] = np.arange(SS['StateMod'].size).reshape(SS['StateMod'].shape)
    SS['sel_feat_idx'] = np.arange(SS['sel_feat_idx'].size).reshape(SS['sel_feat_idx'].shape)
    SS['K'] = np.arange(SS['K'].size).reshape(SS['K'].shape)
    save_decode_params(SS, RootDir)

    
    