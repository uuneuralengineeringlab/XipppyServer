import numpy as np
import xipppy as xp
import time


def get_features(SS):
    """
    SS['buf_len_EMG']: number of EMG samples to pull
    SS['all_EMG_chans']: indices of EMG channels
    SS['num_EMG_chans']: number of EMG channels
    
    returns SS
        SS['feat']: mean absolute value of channels over buffer length
    """
    

    ####### find times and calculate time since last loop finished ###########    
    SS['cur_time'] = xp.time()
    SS['elapsed_time'] = np.float32((SS['cur_time']-SS['prev_time'])/30)
    SS['prev_time'] = SS['cur_time']
    
    ##TODO: add raw 1k EMG data saving continuously. Could be tricky as data is not continuous
    #Starting code for saving raw 1k to file (this should be done in separate thread)
    # d = xp.cont_lfp(37, SS['all_EMG_chans'], 0)
    # d = np.reshape(d[0],(SS['num_EMG_chans'],37)) # want time x channels
    # d[:,0:np.clip(SS['elapsed_time'],0,37)] #save this to file
    # d = d[:,0:SS['buf_len_EMG']]
    
    d = xp.cont_lfp(SS['buf_len_EMG'], SS['all_EMG_chans'], 0)
    d = np.reshape(d[0],(SS['num_EMG_chans'],SS['buf_len_EMG'])) # want time x channels

    
    diffpairs = time.time()
    # diff_pairs = np.dot(SS['EMG_diff_matrix'].T, d)
    # diff_pairs = np.zeros([496,33])
    # print('EMG', SS['EMG_diff_matrix'].T.shape, 'd', d.shape)

    
    buffTime = time.time()
    #only pull 33 samples from xipppy and do diff_pairs on 33
    #build buffer of numChansEMG x 300 here and fill with incoming chunks of 33
    index = SS['EMG_acq_buff_idx']*SS['buf_len_EMG']  #index iterates from 1->10 so 1 * 30 = 30 which is our end index
    SS['prev_EMG_buff'][:,index-SS['buf_len_EMG']:index] = diff_pairs  #place the newest EMG where the oldest EMG is located
    SS['EMG_acq_buff_idx'] += 1 #iterate to keep track of where the oldest emg is
    if SS['EMG_acq_buff_idx'] >= 10: #if its greater than 11 then we cycle back to beginning
        SS['EMG_acq_buff_idx'] = 1 #setting back to beginning
    
    preFeat = time.time()
        
    # mLFP = np.single(np.mean(np.abs(d),axis=1))
    # SS['feat'] = np.single(np.mean(np.abs(diff_pairs),axis=1))
    SS['feat'] = np.single(np.mean(np.abs(SS['prev_EMG_buff']),axis=1))
    
    endLoop = time.time()
    
    print(f'diffpairs: {buffTime - diffpairs:.4f}, buffTime: {preFeat - buffTime:.4f}, preFeat: {endLoop - preFeat:.4f}')
    return SS
