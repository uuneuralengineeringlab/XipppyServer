import numpy as np

def kf_getss_cob(A, W, H, Q): ##codegen
    # klim is a (length(xhat) x 2) matrix with min/max limits for each row in
    # xhat.
    # SteadyState is a 1 or 0 saying whether the gain has converged or not.

    xhat = np.zeros((A.shape[0],1))
    xhat[A.shape[0]-1] = 1; # bias term
    z = np.ones((H.shape[0],1));
    P = np.zeros(A.shape)
    ## Initialize kalman parameters
    # step 1: time-update equations
    xhatm = np.dot(A, xhat) #previous xhat
    Pm = np.dot(np.dot(A, P), A.T) + W ; #previous P #TODO: Mark: this will be zeros + W?
    # ms_log('KF Pm: %d,%d,%d,%d',Pm(1,1),Pm(2,2),Pm(3,3),Pm(4,4))
    # ms_log('KF A: %d,%d,%d,%d',A(1,1),A(2,2),A(3,3),A(4,4))
    # step 2: measurement-update equations
    # K = Pm*H'*pinv(H*Pm*H'+Q);
    K = np.dot(np.dot(Pm, H.T), 
               np.linalg.pinv(np.dot(np.dot(H, Pm), H.T)+Q))
    xhat = xhatm + np.dot(K, (z - np.dot(H, xhatm))) #current xhat
    P = np.dot((np.eye(K.shape[0]) - np.dot(K, H)), Pm) #current P
    delta = 1e-6
    SteadyState = False
    ## Run KF until convergence
    while not SteadyState:
        # step 1: time-update equations
        xhatm = np.dot(A, xhat) #previous xhat
        Pm = np.dot(np.dot(A, P), A.T) + W #previous P
        # step 2: measurement-update equations
        Km = K.copy() # previous K
        K = np.dot(np.dot(Pm, H.T), 
                   np.linalg.pinv(np.dot(np.dot(H, Pm), H.T)+Q)) # current K
        xhat = xhatm + np.dot(K, (z - np.dot(H, xhatm))) #current xhat
        P = np.dot((np.eye(K.shape[0]) - np.dot(K, H)), Pm) #current P
        # Check for all K so see if delta is small (1e-6) if so exit the loop
        if (abs(K-Km)<delta).all():
            SteadyState = True
        
    #     
    # # EMG lpf stream
    #         [dEMG,~] = xl_cont((1:32), 300, 'lfp',0);
    # # EMGPwrMA
    #         dEMGSel = EMGSelMatrix*dEMG;
    #         z =(mean(abs(dEMGSel),2)); #~emg pwr averaged over kernel width = features (1 x chans+diffs)';
    #     
    
    
    #Precalculate the state model update (A-K*H*A)
    StateMod = A - np.dot(np.dot(K, H), A)
    

    return K, StateMod, SteadyState

if __name__ == '__main__':
    from scipy import io
    kalman_mat = io.loadmat(r"C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\HAPTIX-Nomad-Code\EncodeDecode_all files for compile\DummyVars/KalmanMatrices.mat")
    A = kalman_mat['A']
    H = kalman_mat['H']
    W = kalman_mat['W']
    Q = kalman_mat['Q']

    P = np.zeros(A.shape)
    
    # z = io.loadmat(r"C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\HAPTIX-Nomad-Code\EncodeDecode_all files for compile\DummyVars/GramSchmidt_RealZ.mat")
    # z = z['Z']
    
    z = np.ones((528,1))*50
    
    (z, P, A, W, H, Q) = kf_getss_cob(z, P, A, W, H, Q)
    