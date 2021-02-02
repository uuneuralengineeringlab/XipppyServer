import numpy as np


def gramSchmDarpa(X, Z, movements, maximumChannelNumber) -> np.array:
    # This function selects which of a range of possible features best represent the movement data
    #
    # The method used is a forward selection with Gram-Schmidt orthogonalation of both the predictions (movements)
    # and the predictors (features) with every step. In the forward selection process, the feature selected in each
    # step is the one that has linear correlation between the feature and a single movement (DoF).
    #
    # Of note, this Python code assumes 0-based indexing
    #
    # N is number of movements
    # M is number of samples in time - 1
    # K is number of features (estimators) - 1
    # X - matrix as np.array object that contains training movement data, arranged as N by M
    # Z - matrix as np.array object that contains training feature data, arranged as K by M
    # movements - vector as np.array object that contains list of actual movements performed, can be up to M
    # maximumChannelNumber - numeric value containing maximum number of channels that can be selected (1-based)
    #
    # channelSelected - vector as np.array object containing list of index number of feature selected

    # Sanity checks on inputs
    N = X.shape[0]
    M = X.shape[1]
    K = Z.shape[0]
    MZ = Z.shape[1]
    if M != MZ:
        return -1
    if maximumChannelNumber <= 0:
        return -2
    if maximumChannelNumber > K:
        maximumChannelNumber = K
    if np.ndim(movements) > 1:
        movements = movements.ravel()
    if movements.size > N:
        return -3
    if np.max(movements) >= M:
        return -4

    # Force types
    movements = np.around(movements, 0).astype(int)
    maximumChannelNumber = int(round(maximumChannelNumber))

    # define output, use -1 to flag error
    channelSelected = np.empty(maximumChannelNumber)
    channelSelected.fill(-1)

    # Possible channels
    possibleChannels = np.arange(K)

    # Extract working copy of Z and center
    Zfull = np.copy(Z[possibleChannels, :])
    z = np.isnan(Zfull)
    Zfull[z] = 0
    #  Zfull[np.isnan(Zfull)] = 0
    #  Zfull = np.where( np.isnan(Zfull), 0, Zfull );
    Zfull -= np.mean(Zfull, axis=1, keepdims=True)
    #  Zfull.tofile('ZfullOriginal.bin')

    # Extract working copy of X and center
    Xfull = X[movements, :].copy()
    Xfull -= np.mean(Xfull, axis=1, keepdims=True)
    #  Xfull.tofile('Xfull.bin')

    # Update sizes
    N = Xfull.shape[0]
    M = Xfull.shape[1]
    K = Zfull.shape[0]

    # Initialize for looping
    XfullEstimate = np.zeros([N, M])  # running estimate of Xfull from selected features
    channelSelected = np.empty([maximumChannelNumber, 1], dtype=int)
    # list of active channels, -1 is flag for not filled yet
    channelSelected.fill(-1)
    Zfull /= np.linalg.norm(Zfull, axis=1, keepdims=True)
    chanStatus = np.ones([K, 1])

    # Loop through all best feature channels
    for j in range(maximumChannelNumber):

        # Loop status
        #  print('Loop = ', j )

        # Find residual
        XfullResidual = Xfull - XfullEstimate
        XfullResidualNorm = XfullResidual/np.linalg.norm(XfullResidual, axis=1, keepdims=True)

        # find correlation
        corr = Zfull @  XfullResidualNorm.T
        corr = np.abs(corr)
        if j > 0:
            corr[channelSelected[0:j-1], :] = np.NAN
        #  corr.tofile('corr.bin')

        # Extract best feature channel
        corrmax = np.nanmax(corr, axis=1)
        #  corrmax.tofile('corrmax.bin')
        ind = np.nanargmax(corrmax)
        channelSelected[j] = ind
        indNot = np.delete(np.arange(K), ind)
        #  print('Selected channel ', ind)
        #  print('Best corrcoef = ', corrmax[ind])

        # Work with best feature
        selectedFeatureData = np.reshape(Zfull[ind, :], [-1, 1])
        chanStatus[ind] = 0

        # Estimate residuals movements with selected feature and to running sum of movements
        betaXfullTuple = np.linalg.lstsq(selectedFeatureData, XfullResidual.T, rcond=None)
        betaXfull = (betaXfullTuple[0]).T
        #  print(betaXfull)
        #  betaXfull.tofile('betaXfull.bin')
        #  print(selectedFeatureData.shape)
        #  print(betaXfull.shape)
        XfullEstimate += betaXfull @ selectedFeatureData.T

        # Estimate features with selected feature and remove from features
        betaZfull = np.zeros([K, 1])
        betaZfullTuple = np.linalg.lstsq(selectedFeatureData, Zfull[indNot, :].T, rcond=None)
        #  print(betaZfullTuple)
        betaZfull[indNot] = (betaZfullTuple[0]).T
        #  betaZfull.tofile('betaZfull.bin')
        Zfull -= betaZfull @ selectedFeatureData.T
        for k in range(K):
            if chanStatus[k] == 1:
                Zfull[k, :] /= np.linalg.norm(Zfull[k, :])
                if np.any(np.isnan(Zfull[k, :])):
                    Zfull[k, :] = 0
            else:
                Zfull[k, :] = 0

        #  Zfull.tofile('Zfull.bin')
        #  XfullEstimate.tofile('XfullEstimate.bin')
    
    return channelSelected


if __name__ == '__main__':
    # not very good practice, but only need to import os and scipy for testing
    import os
    import scipy as sp
    import timeit
    # from .scipy import io
    if os.path.isfile(r'C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\EncodeDecode_all files for compile\DummyVars\GramSchmidt_RealX.mat'):
        X = sp.io.loadmat(r'C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\EncodeDecode_all files for compile\DummyVars\GramSchmidt_RealX.mat')
        X = X['X']
        Z = sp.io.loadmat(r"C:\Users\Administrator\Code\COB\COB_DevFolder\xl_build_code\EncodeDecode_all files for compile\DummyVars\GramSchmidt_RealZ.mat")
        Z = Z['Z']
        movements = np.argwhere(np.any(X,axis=1))
    else:
        X = np.random.rand(6, 5000)
        Z = np.random.rand(100, 5000)
        movements = np.arange(6)
    maximumChannelNumber = 48
    t = timeit.Timer(lambda: gramSchmDarpa(X, Z, movements, maximumChannelNumber))
    print('Time:', t.timeit(number=1))
    channelSelected = gramSchmDarpa(X, Z, movements, maximumChannelNumber)
    print(channelSelected)
