import numpy as np

# def kf_test_cob(xhat, z, K,StateMod, klim): ##codegen
def kf_test_cob(SS): ##codegen
    # klim is a (length(xhat) x 2) matrix with min/max limits for each row in
    # xhat.
    # xhatm = TRAIN.A*xhat; 
    # xhat = xhatm+TRAIN.K*(z-TRAIN.H*xhatm);
    #
    # adapted from imprtKalman_testSS_mod.m
    if SS['train_kf_phase'] == None: # only run Kalman if not training
        z = SS['feat'][SS['sel_feat_idx']].reshape(-1,1)
        SS['xhat_raw'] = np.dot(SS['StateMod'], SS['xhat_raw']) + np.dot(SS['K'], z)
        
        # bound xhats to klim
        SS['xhat_raw'] = np.clip(SS['xhat_raw'], -1, 1)
        SS['xhat'] = SS['xhat_raw'].copy()
    
    return SS

# if __name__ == '__main__':
    # from scipy import io
    # KalmanParams = io.loadmat(r"C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\HAPTIX-Nomad-Code\EncodeDecode_all files for compile\DummyVars\KalmanTest.mat")
    # K = KalmanParams['K']
    # klim = KalmanParams['klim']
    # xhat = KalmanParams['xhat']
    # z = KalmanParams['z']
    # StateMod = KalmanParams['StateMod']
    