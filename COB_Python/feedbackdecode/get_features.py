import numpy as np
import xipppy as xp


def get_features(SS):
    """
    SS['bufLenEMG']: number of EMG samples to pull
    SS['chanListEMG']: indices of EMG channels
    SS['numChansEMG']: number of EMG channels
    
    returns SS
        SS['feat']: mean absolute value of channels over buffer length
    """
    

    ####### find times and calculate time since last loop finished ###########
    SS['curTime'] = np.float64(xp.time())
    SS['elapsedTime'] = (SS['curTime']-SS['prevTime'])/30
    SS['prevTime'] = SS['curTime']
    
    ##TODO: add raw 1k EMG data saving continuously. Could be tricky as data is not continuous
    #Starting code for saving raw 1k to file (this should be done in separate thread)
    # d = xp.cont_lfp(37, SS['chanListEMG'], 0)
    # d = np.reshape(d[0],(SS['numChansEMG'],37)) # want time x channels
    # d[:,0:np.clip(SS['elapsedTime'],0,37)] #save this to file
    # d = d[:,0:SS['bufLenEMG']]
    
    d = xp.cont_lfp(SS['bufLenEMG'], SS['chanListEMG'], 0)
    d = np.reshape(d[0],(SS['numChansEMG'],SS['bufLenEMG'])) # want time x channels
        
    
    diff_pairs = np.dot(SS['EMG_diff_matrix'].T, d)
    
    #only pull 33 samples from xipppy and do diff_pairs on 33
    #build buffer of numChansEMG x 300 here and fill with incoming chunks of 33
    index = SS['iterationBuffCounter']*SS['bufLenEMG']  #index iterates from 1->10 so 1 * 30 = 30 which is our end index
    SS['previousEMGbuff'][:,index-SS['bufLenEMG']:index] = diff_pairs  #place the newest EMG where the oldest EMG is located
    SS['iterationBuffCounter'] += 1 #iterate to keep track of where the oldest emg is
    if SS['iterationBuffCounter'] >= 10: #if its greater than 11 then we cycle back to beginning
        SS['iterationBuffCounter'] = 1 #setting back to beginning
        
    # mLFP = np.single(np.mean(np.abs(d),axis=1))
    # SS['feat'] = np.single(np.mean(np.abs(diff_pairs),axis=1))
    SS['feat'] = np.single(np.mean(np.abs(SS['previousEMGbuff']),axis=1))
    
    return SS
