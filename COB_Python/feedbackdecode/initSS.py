CSF, CSA = 10, 20
import numpy as np
import feedbackdecode as fd
import xipppy as xp
import copy

def initSS():
    ''' 
    This function defines all fields of SS dict before it is read or used.
    
    Adapted from initSS.m
    '''
    
    SS = {}
    
    #~~~~~~~~~~~~~~~~~~~~~~~~~~ From Current XipppyServer ~~~~~~~~~~~~~~~~~~~~#
    ######################### Timing items ###################################
    SS['cur_time'] = np.zeros(1, dtype=float) # current time in loop
    SS['prev_time'] = np.zeros(1, dtype=float) # previous time in loop
    SS['calc_time'] = np.zeros(1, dtype=float) # time for calculations in loop
    SS['elapsed_time'] = np.zeros(1, dtype=float) # time between loop cycles
    
    
    ################## General File I/O, System setup ########################
    SS['eventparams_fid'] = None
    SS['eyn_fid'] = None
    SS['neural_FE_idx'] = [0, 32, 64, 128, 160, 192, 256, 288, 320]
    SS['all_neural_chans'] = np.hstack((np.arange(96), np.arange(128,128+96), 
                                        np.arange(256, 256+96)))
    
    ########################## Decode items ##################################
    SS['num_EMG_chans'] = int(32) # number of EMG channels
    SS['EMG_diff_matrix'] = fd.genEMGDiffMatrix_simple(SS['num_EMG_chans'])
    SS['bad_EMG_elecs'] = np.array([], dtype=int) # electrodes themselves
    # SS['bad_EMG_elecs'] = np.arange(20) # electrodes themselves
    SS['bad_EMG_chans'] = np.array([]) # which channels (from EMG_diff_matrix) should be excluded. Calculated in fd.find_bad_chans
    SS['num_diff_pairs'] = int(SS['EMG_diff_matrix'].shape[1]) # number of differentials
    SS['num_features'] = int(48) # channels to use after channel selection
    SS['buf_len_EMG'] = int(33) # number of samples for moving average (1 kHz)
    SS['prev_EMG_buff'] = np.zeros((SS['num_diff_pairs'],297), dtype=np.single)  #Holds the values of all the previous 
    SS['all_EMG_chans'] = np.arange(384, 384+SS['num_EMG_chans'], dtype=int) # Port C: 256, Port D: 384
    SS['feat'] = np.zeros(SS['num_diff_pairs'], dtype=np.single) # EMG features, MAV of LFP data # 
    SS['sel_feat_idx'] = np.arange(SS['num_features'], dtype=int) # Channel selected features
    SS['train_iter'] = None # iterator for training through WTS file 
    SS['train_seq'] = np.zeros((7,33*5)) # training sequence (iterated by train_iter), DOFs x sequence
    SS['kin'] = np.zeros(7, dtype=np.single) # current kinematics in training sequence
    SS['train_fid'] = None # file identifier for writing training .kdf file
    SS['xhat'] = np.zeros((7,1), dtype=np.single) # predicted kinematics (bias term is 7th value)
    SS['xhat_raw'] = np.zeros((7,1), dtype=np.single) # predicted kinematics unaffected by threshold or latch (bias term is 7th value)
    SS['threshold'] = 0.1 # only a global threshold will be applied, chose 0.1 because we also use latching filter
    SS['K'] = np.zeros((7,SS['num_features']))
    SS['StateMod'] = np.zeros((7,7))
    SS['klim'] = np.c_[-np.ones((7,1)), np.ones((7,1))]
    SS['xhat_prev'] = np.zeros((7,1))
    SS['train_kf_phase'] = None # automatically load previous training
    SS['EMG_acq_buff_idx'] = 1
    SS['wrist_mode'] = 0 # velocity=0, position=1 
    SS['lock_DOF'] = np.zeros(7, dtype=bool)
    SS['stop_hand'] = 1
    SS['mirror_DOF'] = np.zeros(7, dtype=int)
    SS['chan_sel_queue'] = None # for running channel selection on another core
    SS['chan_sel_proc'] = None # for running channel selection on another core
    SS['train_feat'] = None # populated when reading .kdf for training kalman
    SS['train_kin'] = None # populated when reading .kdf for training kalman

    
    
    ############################ Encode items ################################
    SS['cur_sensors'] = np.zeros(19, dtype=np.single) # 19 sensors from deka
    SS['past_sensors'] = np.zeros((19,5), dtype=np.single) # 19 sensors from deka
    
    #stim_params: chan,sensor_idx,encode_alg,minamp,maxamp,minfreq,maxfreq,enabled (experimenter), enabled (user) (see DEKA2StimCOB for details)
    SS['stop_stim'] = 1
    SS['active_stim'] = np.array([]).reshape((0,7)) # just needs the first 7 parameters from 'stim_params'
    SS['stim_params'] = np.array([]).reshape((0,9)) # this is the default. Below for testing.
    # SS['stim_params'] = np.array([1,0,3,0,10,0,100,0,1]) 
    # SS['stim_params'] = np.array([[223,2,3,30,30,0,300,1,1],
    #                               [1,1,3,0,10,0,300,1,1],
    #                               [2,2,3,0,10,0,300,1,0],
    #                               [3,3,3,0,10,0,300,1,1],
    #                               [4,4,3,0,10,0,300,1,1]])
    # SS['stim_params'] = np.array([[ 0, 0,3,0,10,30,300,1,1],
    #                               [ 1, 1,3,0,10,30,300,1,1],
    #                               [ 2, 2,3,0,10,30,300,1,1],
    #                               [ 3, 3,3,0,10,30,300,1,1],
    #                               [ 4, 4,3,0,10,30,300,1,1],
    #                               [ 5, 5,3,0,10,30,300,1,1],
    #                               [ 6, 6,3,0,10,30,300,1,1],
    #                               [ 7, 7,3,0,10,30,300,1,1],
    #                               [ 8, 8,3,0,10,30,300,1,1],
    #                               [ 9, 9,3,0,10,30,300,1,1],
    #                               [10,10,3,0,10,30,300,1,1],
    #                               [11,11,3,0,10,30,300,1,1],
    #                               [12,12,3,0,10,30,300,1,1],
    #                               [13,13,3,0,10,30,300,1,1],
    #                               [14,14,3,0,10,30,300,1,1],
    #                               [15,11,3,0,10,30,300,1,1],
    #                               [16,12,3,0,10,30,300,1,1],
    #                               [17,13,3,0,10,30,300,1,1],
    #                               [18,14,3,0,10,30,300,1,1],
    #                               [19,14,3,0,10,30,300,1,1],
    #                               [20,11,3,0,10,30,300,1,1],
    #                               [21,12,3,0,10,30,300,1,1],
    #                               [22,13,3,0,10,30,300,1,1],
    #                               [23,14,3,0,10,30,300,1,1],
    #                               [24,14,3,0,10,30,300,1,1],
    #                               [25,11,3,0,10,30,300,1,1],
    #                               [26,12,3,0,10,30,300,1,1],
    #                               [27,13,3,0,10,30,300,1,1],
    #                               [28,14,3,0,10,30,300,1,1],
    #                               [29,11,3,0,10,30,300,1,1],
    #                               [30,12,3,0,10,30,300,1,1],
    #                               [31,13,3,0,10,30,300,1,1],
    #                               [32,14,3,0,10,30,300,1,1],
    #                               [33,14,3,0,10,30,300,1,1],
    #                               [34,11,3,0,10,30,300,1,1],
    #                               [35,12,3,0,10,30,300,1,1],
    #                               [36,13,3,0,10,30,300,1,1],
    #                               [37,14,3,0,10,30,300,1,1],
    #                               [38,14,3,0,10,30,300,1,1],
    #                               [222,11,3,0,10,30,300,1,1],
    #                               [223,12,3,0,10,30,300,1,1],
    #                               [350,13,3,0,10,30,300,1,1],
    #                               [351,14,3,0,10,30,300,1,1]])
    
    elec = 1
    period = 0
    repeats = 0
    stim_segs = []
    stim_segs.append(xp.StimSegment(length=6, amplitude=50, polarity=-1))
    stim_segs.append(xp.StimSegment(length=3, amplitude=0,   polarity=-1))
    stim_segs.append(xp.StimSegment(length=6, amplitude=50, polarity= 1))
    SS['stim_cmd'] = xp.StimSeq(elec, period, repeats, *stim_segs, action=1) # action 1='curcyc', 0='immed'
    # for i in range(SS['active_stim'].shape[0]): #this seems dynamic enough between stuff?
    SS['stim_seq'] = [] # list of Ripple's StimSeq class (for each chan)
    for _ in range(50): # upper limit of stim channels right now is 50
        SS['stim_seq'].append(copy.deepcopy(SS['stim_cmd']))
        
    SS['next_pulse'] = np.zeros((96*3))
    SS['stim_freq_save'] = np.zeros(96*3) # save stim freq for two USEAs (0-95, 128-223, 256-351)
    SS['stim_amp_save'] = np.zeros(96*3) # save stim amp for two USEAs
    # SS['active_stim'] = np.array([], dtype=bool)
    # SS['ContStimAmp'] = None
    # SS['ContStimFreq'] = None    
    
    return SS