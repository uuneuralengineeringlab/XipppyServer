import numpy as np

def kf_train_cob(x,z): ##codegen
    """
    x: kinematics (samples, DOF)
    z: features (samples, features)
    """
    
    # import pdb; pdb.set_trace()
    # some dimension checking
    if x.shape[0] > x.shape[1]:
        x = x.T
    if z.shape[0] > z.shape[1]:
        z = z.T
        

    # calculate mu's, the means of kinematics and features, and subtract off
    N = x.shape[0] # numDOFs
    # F = size(z,1); # numFeatures
    # xmu=zeros(N,1); #mean(x,2); x=x-repmat(PARAM.xmu,[1 size(x,2)]);
    # zmu=zeros(F,1); #mean(z,2); z=z-repmat(PARAM.zmu,[1 size(z,2)]);
    # length of the signal
    M = x.shape[1]
    # init structure
    # TRAIN = struct('A',zeros(N,N),'W',zeros(N,N),'Pzx',zeros(F,N),'Rxx',zeros(N,N),...
    #     'Rzz',zeros(F,F),'H',zeros(F,N),'Q',zeros(F,F),'N',N);
    
    # calculate A, the state-to-state transformation (hand kinematics)
    A1 = np.dot(x[:,1:M], x[:,0:(M-1)].T)
    A2 = np.dot(x[:,0:(M-1)], x[:,0:(M-1)].T)
    A = np.dot(A1, np.linalg.pinv(A2))

    # calculate W, the covariance of the noise in the kinematics
    W1 = np.dot(x[:,1:M], x[:,1:M].T)
    W2 = np.dot(x[:,0:(M-1)], x[:,1:M].T)
    W = (1 / (M - 1)) * (W1 - np.dot(A, W2))
    
    # cross-correlation and autocorrelations of x and z
    Pzx = np.dot(z[:,0:M], x[:,0:M].T)
    Rxx = np.dot(x[:,0:M], x[:,0:M].T)
    Rzz = np.dot(z[:,0:M], z[:,0:M].T)
    
    # calculate H, the transformation matrix from measured features to state
    H = np.dot(Pzx, np.linalg.pinv(Rxx))

    # calculate Q, the covariance of noise in the measured features
    Q = (1 / M) * (Rzz - np.dot(H, Pzx.T))
    
    return (A,W,Pzx,Rxx,Rzz,H,Q,N)


if __name__ == '__main__':
    import scipy as sp
    from scipy import io
    x = io.loadmat(r"C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\HAPTIX-Nomad-Code\EncodeDecode_all files for compile\DummyVars/GramSchmidt_RealX.mat")
    x = x['X']
    z = io.loadmat(r"C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\HAPTIX-Nomad-Code\EncodeDecode_all files for compile\DummyVars/GramSchmidt_RealZ.mat")
    z = z['Z']
    