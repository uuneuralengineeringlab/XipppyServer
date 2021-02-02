import numpy as np
import feedbackdecode as fd
import xipppy as xp
import copy

def initSS():
    ''' This function defines all fields of SS dict before it is read or used.
    
    Adapted from initSS.m
    '''
    
    SS = {}
    
    #~~~~~~~~~~~~~~~~~~~~~~~~~~ From Current XipppyServer ~~~~~~~~~~~~~~~~~~~~#
    ######################### Timing items ###################################
    SS['curTime'] = np.zeros(1, dtype=float) # current time in loop
    SS['prevTime'] = np.zeros(1, dtype=float) # previous time in loop
    SS['calcTime'] = np.zeros(1, dtype=float) # time for calculations in loop
    SS['elapsedTime'] = np.zeros(1, dtype=float) # time between loop cycles
    
    
    ######################### General File I/O ###############################
    SS['eventparams_fid'] = None
    SS['eyn_fid'] = None
    
    ########################## Decode items ##################################
    SS['numChansEMG'] = int(32) # number of EMG channels
    SS['EMG_diff_matrix'] = fd.genEMGDiffMatrix_simple(SS['numChansEMG'])
    SS['bad_EMG_elecs'] = np.array([], dtype=int) # electrodes themselves
    # SS['bad_EMG_elecs'] = np.arange(20) # electrodes themselves
    SS['bad_EMG_chans'] = np.array([]) # which channels (from EMG_diff_matrix) should be excluded. Calculated in fd.find_bad_chans
    SS['numDiffPairs'] = int(SS['EMG_diff_matrix'].shape[1]) # number of differentials
    SS['numFeatures'] = int(48) # channels to use after channel selection
    SS['bufLenEMG'] = int(33) # number of samples for moving average (1 kHz)
    SS['previousEMGbuff'] = np.zeros((SS['numDiffPairs'],297), dtype=np.single)  #Holds the values of all the previous 
    SS['chanListEMG'] = np.arange(256, 256+SS['numChansEMG'], dtype=int) # Port C: 256, Port D: 384
    SS['feat'] = np.zeros(SS['numDiffPairs'], dtype=np.single) # EMG features, MAV of LFP data # 
    SS['feat_idx'] = np.arange(SS['numFeatures'], dtype=int) # Channel selected features
    SS['train_iter'] = None # iterator for training through WTS file 
    SS['train_seq'] = np.zeros((7,33*5)) # training sequence (iterated by train_iter), DOFs x sequence
    SS['kin'] = np.zeros(7, dtype=np.single) # current kinematics in training sequence
    SS['train_fid'] = None # file identifier for writing training .kdf file
    SS['xhat'] = np.zeros((7,1), dtype=np.single) # predicted kinematics (bias term is 7th value)
    SS['xhat_raw'] = np.zeros((7,1), dtype=np.single) # predicted kinematics unaffected by threshold or latch (bias term is 7th value)
    SS['threshold'] = 0.1 # only a global threshold will be applied, chose 0.1 because we also use latching filter
    SS['K'] = np.zeros((7,SS['numFeatures']))
    SS['StateMod'] = np.zeros((7,7))
    SS['klim'] = np.c_[-np.ones((7,1)), np.ones((7,1))]
    SS['xhat_prev'] = np.zeros((7,1))
    SS['train_kf_phase'] = None # automatically load previous training
    SS['iterationBuffCounter'] = 1
    SS['WristMode'] = 0 # velocity=0, position=1 
    SS['LockDOF'] = np.zeros(7, dtype=bool)
    SS['stopHand'] = 0
    SS['MirrorDOF'] = np.zeros(7, dtype=int)
    SS['chan_sel_queue'] = None # for running channel selection on another core
    SS['chan_sel_proc'] = None # for running channel selection on another core
    SS['train_feat'] = None # populated when reading .kdf for training kalman
    SS['train_kin'] = None # populated when reading .kdf for training kalman

    
    
    ############################ Encode items ################################
    SS['curSensors'] = np.zeros(19, dtype=np.single) # 19 sensors from deka
    SS['pastSensors'] = np.zeros((19,5), dtype=np.single) # 19 sensors from deka
    
    # Note: SS['StimChan'] must be typecasted from a list to be iterable
    # SS['StimChan'] = np.array([0]) # Dynamically changes to number of stim channels; leave as np.array([]) unless debugging
    
    # StimChan = SS['StimParams'][k,0] #nomad channel (0-96, 128)
    # DEKARegion = SS['StimParams'][k,1] #index into DEKASensors
    # EncodeAlg = SS['StimParams'][k,2] #to determine encoding algorithm
    # MinAmp = SS['StimParams'][k,3]
    # MaxAmp = SS['StimParams'][k,4]
    # MinFreq = SS['StimParams'][k,5]
    # MaxFreq = SS['StimParams'][k,6]
    
    #replaces StimChan and includes all parameters
    #StimParams: chan,sensor_idx,encode_alg,minamp,maxamp,minfreq,maxfreq,enabled (experimenter), enabled (user) (see DEKA2StimCOB for details)
    SS['active_stim'] = np.array([]).reshape((0,7)) # just needs the first 7 parameters from 'StimParams'
    SS['StimParams'] = np.array([]).reshape((0,9)) # this is the default. Below for testing.
    # SS['StimParams'] = np.array([1,0,3,0,10,0,100,0,1]) 
    # SS['StimParams'] = np.array([[15,2,3,30,30,0,300,0,1],
    #                               [1,1,3,0,10,0,300,1,1],
    #                               [2,2,3,0,10,0,300,1,0],
    #                               [3,3,3,0,10,0,300,1,1],
    #                               [4,4,3,0,10,0,300,1,1]])
    # SS['StimParams'] = np.array([[ 0, 0,3,0,10,30,300,1,1],
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
    #                               [39,11,3,0,10,30,300,1,1],
    #                               [40,12,3,0,10,30,300,1,1],
    #                               [41,13,3,0,10,30,300,1,1],
    #                               [42,14,3,0,10,30,300,1,1]])
    
    elec = 1
    period = 0
    repeats = 0
    stim_segs = []
    stim_segs.append(xp.StimSegment(length=6, amplitude=50, polarity=-1))
    stim_segs.append(xp.StimSegment(length=3, amplitude=0,   polarity=-1))
    stim_segs.append(xp.StimSegment(length=6, amplitude=50, polarity= 1))
    SS['StimCmd'] = xp.StimSeq(elec, period, repeats, *stim_segs, action=1) # action 1='curcyc', 0='immed'
    # for i in range(SS['active_stim'].shape[0]): #this seems dynamic enough between stuff?
    SS['StimSeq'] = [] # list of Ripple's StimSeq class (for each chan)
    for i in range(50):
        SS['StimSeq'].append(copy.deepcopy(SS['StimCmd']))
        
    SS['NextPulse'] = np.zeros((96))
    # SS['active_stim'] = np.array([], dtype=bool)
    # SS['ContStimAmp'] = None
    # SS['ContStimFreq'] = None    
    

    #~~~~~~~~~~~~~~~~~~~~~~~~~~ End fields from XipppyServer ~~~~~~~~~~~~~~~~~#
    
    
    ################ Mark's original variables ###############################
    # #### debug mode
    # SS['Debug'] = True
    # SS['Toggler'] = True
    
    # #### NIP and code
    # # NumStimChansAvail = 96 # only 96 on Port A
    # NumStimChansAvail = 128 # to test 96 on port A and 32 on port B
    # NumEMGChansAvail = 32
    # EMGChanStart = 257 # port C 
    # StimChanStart = 1 # start with port A ## TODO: may need to change for xipppy call
    # durTotal = 300 # moving average window (buffer)
    # maxdur = 50 # maximum samples to be collected within each loop
    # SS['BaseLoopTime'] = 0.0330
    # SS['TCur'] = np.zeros((1,1))
    # SS['T2'] = np.zeros((1,1))
    # SS['Run'] = True
    # SS['Nowfolder'] = 'empty'
    # #coder.varsize('SS.Nowfolder',[1,90],[0,1])

    # #### Decode training
    # SS['WTSfile'] = 'Default'
    # #coder.varsize('SS.WTSfile',[1,60],[0,1])
    # SS['ToLoadWTS'] = False
    
    # #### Decode
    # SS['ToLoadKTF'] = False
    # SS['ToLoadDPF'] = False
    # SS['MotorOn'] = 1
    
    # SS['KTFfile'] = 'Default'
    # #coder['varsize('SS['KTFfile',[1,60],[0,1])
    # SS['DPFfile'] = 'Default'
    # #coder['varsize('SS['DPFfile',[1,60],[0,1])
    # SS['mKDFfile'] = ''
    # #coder['varsize('SS['mKDFfile',[1,60],[0,1])
    # SS['DEKAfile'] = ''
    # #coder['varsize('SS['DEKAfile',[1,60],[0,1])
    # SS['EMGfile'] = ''
    # #coder['varsize('SS['EMGfile',[1,60],[0,1])
    # SS['mKDFFID'] = -1
    # SS['dekaFID'] = -1
    # SS['emg1kFID'] = -1
    # SS['DecodeLoaded'] = False
    
    
    # maxDOFs = 7  # fixed: DEKA 6 + Bias term for KF
    # maxFeatures = 48 # change if want, but it will add time required to compute training
    # SS['numDOFs'] = maxDOFs
    # SS['KernelWidth'] = 0.300
    # SS['A'] = np.zeros((maxDOFs,maxDOFs))
    # #coder['varsize('SS['A',[maxDOFs,maxDOFs],[1,1])
    # SS['W'] = np.zeros((maxDOFs,maxDOFs))
    # #coder['varsize('SS['W',[maxDOFs,maxDOFs],[1,1])
    # SS['H'] = np.zeros((maxFeatures,maxDOFs))
    # #coder['varsize('SS['H',[maxFeatures,maxDOFs],[1,1])
    # SS['Q'] = np.zeros((maxFeatures,maxFeatures))
    # #coder['varsize('SS['Q',[maxFeatures,maxFeatures],[1,1])
    # SS['P'] = np.zeros((maxDOFs,maxDOFs))
    # #coder['varsize('SS['P',[maxDOFs,maxDOFs],[1,1])
    # SS['KalmanGain'] = np.ones((maxDOFs,2))
    # #coder['varsize('SS['KalmanGain',[maxDOFs,2],[1,0])
    # SS['KalmanThresh'] = np.ones((maxDOFs,2))*0.2
    # #coder['varsize('SS['KalmanThresh',[maxDOFs,2],[1,0])
    # SS['CtrlSpeed'] = np.ones((maxDOFs,1))
    # #coder['varsize('SS['CtrlSpeed',[maxDOFs,1],[1,0])
    # SS['CtrlMode'] = np.ones((maxDOFs,1))   #### This is the control mode for the mKF: 1 = position 2 = latching 4 = leaky
    # #coder['varsize('SS['CtrlMode',[maxDOFs,1],[1,0])
    
    # SS['KalmanIdxs'] = np.zeros((maxFeatures,1))
    # #coder['varsize('SS['KalmanIdxs',[maxFeatures,1],[1,0])
    # SS['KalmanMvnts'] = np.zeros((1,maxDOFs))
    # #coder['varsize('SS['KalmanMvnts',[1,maxDOFs],[0,1])
    # SS['AvailEMG'] = np.arange(EMGChanStart,EMGChanStart+NumEMGChansAvail) ## TODO: may need to change for xipppy 
    # #coder['varsize('SS['AvailEMG',[1,NumEMGChansAvail],[0,1])
    # SS['AvailNeural'] = np.arange(StimChanStart,StimChanStart+NumStimChansAvail) ## TODO: may need to change for xipppy
    # #coder['varsize('SS['AvailNeural',[1,NumStimChansAvail],[0,1])
    # SS['BadEMGChans'] = np.zeros((1,1)) ## NOT YET IMPLEMENTED
    # #coder['varsize('SS['BadEMGChans',[1,NumEMGChansAvail],[0,1])
    # SS['EMGBaselineData'] = np.zeros(((maxFeatures),1)) ## NOT IMPLEMENTED - use bias term instead
    # #coder['varsize('SS['EMGBaselineData',[maxFeatures,1],[1,0])
    # SS['K'] = np.zeros((maxDOFs,maxFeatures))
    # #coder['varsize('SS['K',[maxDOFs,maxFeatures],[1,1])
    # SS['SM'] = np.zeros((maxDOFs,maxDOFs))
    # #coder['varsize('SS['SM',[maxDOFs,maxDOFs],[1,1])
    # # SS['DEKAMvnts = false(maxDOFs,1)
    # SS['DEKAMvnts'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['DEKAMvnts',[maxDOFs,1],[1,0])
    # # SS['CtrlModeVel = np.ones((1,maxDOFs)*2  ## CtrlMode for the case when all DOFs in velocity
    # # #coder['varsize('SS['CtrlModeVel',[1,maxDOFs],[0,1])
    # SS['CtrlModeVel'] = np.ones((maxDOFs,1))*2  ## CtrlMode for the case when all DOFs in velocity
    # #coder['varsize('SS['CtrlModeVel',[maxDOFs,1],[1,0])
    # SS['PosVel'] = np.zeros((maxDOFs,1))  ## Per the DEKA PI Interface: velocity = 1, position = 0 - we normally run with both wrists in velocity mode per PI interface
    # #coder['varsize('SS['PosVel',[maxDOFs,1],[1,0])
    # SS['allVel'] = 0
    # SS['restPos'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['restPos',[maxDOFs,1],[1,0])
    # SS['fixedDOFs'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['fixedDOFs',[maxDOFs,1],[1,0])
    # SS['FixPosition'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['FixPosition',[maxDOFs,1],[1,0])
    # SS['EMGKalmanIdxs'] = np.zeros((maxFeatures,1))
    # #coder['varsize('SS['EMGKalmanIdxs',[maxFeatures,1],[1,0])
    # SS['EMGSelMatrix'] = np.zeros((NumEMGChansAvail,maxFeatures))
    # #coder['varsize('SS['EMGSelMatrix',[NumEMGChansAvail,maxFeatures],[1,1])
    # SS['LinkedDOF'] = [0]
    # #coder['varsize('SS['LinkedDOF',[1,1],[1,1])
    
    # SS['xhat'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['xhat',[maxDOFs,1],[1,0])
    # SS['xhat2'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['xhat2',[maxDOFs,1],[1,0])
    # SS['neutral'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['neutral',[maxDOFs,1],[1,0])
    # SS['xhat_all'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['xhat_all',[maxDOFs,1],[1,0])
    # SS['xhatDEKA'] = np.zeros((maxDOFs,1))
    # #coder['varsize('SS['xhatDEKA',[maxDOFs,1],[1,0])
    # SS['Z'] = np.zeros((maxFeatures,1))
    # #coder['varsize('SS['Z',[maxFeatures,1],[1,0])
    # SS['SteadyStateKF'] = False
    # SS['LF_C'] = 1
    # SS['BufferInd'] = np.arange(durTotal) ## TODO: may cause problems starting from 0 not 1
    # SS['dEMG'] = np.zeros((NumEMGChansAvail,maxdur))
    # #coder['varsize('SS['dEMG',[NumEMGChansAvail,maxdur],[1,1])
    # SS['EMGTime_start'] = 0
    # SS['EMGTime_end'] = 0
    # SS['EMGBuffer'] = np.zeros((maxFeatures,durTotal))
    # #coder['varsize('SS['EMGBuffer',[maxFeatures,durTotal],[1,0])
    # SS['MapType'] = {}
    # SS['MapType']['EMG'] = 'passive' ### NOT IMPLEMENTED EMG Mapping
    # #coder['varsize('SS['MapType['EMG',[1,7],[0,1])
    # SS['EventButton'] = np.int32(0)
    
    # #### Stim
    # SS['StimOn'] = 0
    
    # SS['ToLoadEPF'] = False
    # SS['EPFfile'] = 'Default'
    # #coder['varsize('SS['EPFfile',[1,60],[0,1])
    
    # SS['StimElec'] = np.zeros((0,1))
    # #coder['varsize('SS['StimElec',[NumStimChansAvail,1],[1,0])
    # SS['StimChan'] = np.zeros((0,1))
    # #coder['varsize('SS['StimChan',[NumStimChansAvail,1],[1,0])
    # SS['StimElecTest'] = np.zeros((0,1))
    # #coder['varsize('SS['StimElecTest',[NumStimChansAvail,1],[1,0])
    # SS['StimLoaded'] = False
    # SS['NumStimChansAvail'] = NumStimChansAvail ### number of chans to stim on
    # SS['StimCmd'] = {
    #         'elec':0,
    #         'period':0,
    #         'repeats':0,
    #         'action':'curcyc',
    #         'seq':{'length':{6,3,6},
    #                'ampl':{0,0,0},
    #                'pol':{0,0,1},
    #                'fs':{0,0,0},
    #                'enable':{1,0,1},
    #                'delay':{0,0,0},
    #                'ampSelect':{1,1,1}
    #                 }
    #         }  ## xippmex stimseq command
    # #coder.varsize('SS['StimCmd['action', [1 6],[0 1])
    # SS['NextPulse'] = np.zeros((NumStimChansAvail,1)) #holds next pulse time for each stim channel
    
    # SS['StimArray'] = np.zeros((0,8)) # empty because it was variable
    # #coder['varsize('SS['StimArray',[NumStimChansAvail,8],[1,0])
    # SS['Elecs'] = np.zeros((NumStimChansAvail,1)) # Each electrode is tied to a DEKA sensor in the DEKAMapping field
    # #coder['varsize('SS['Elecs',[NumStimChansAvail,1],[1,0])
    # SS['DEKAMapping'] = np.zeros((NumStimChansAvail,1)) # Each sensor is tied to one or more electrodes with this field.
    # #coder['varsize('SS['DEKAMapping',[NumStimChansAvail,1],[1,0])
    # SS['PWDur'] = np.zeros((NumStimChansAvail,1))
    # #coder['varsize('SS['PWDur',[NumStimChansAvail,1],[1,0])
    # SS['AmpMax'] = np.zeros((NumStimChansAvail,1))
    # #coder['varsize('SS['AmpMax',[NumStimChansAvail,1],[1,0])
    # SS['AmpMin'] = np.zeros((NumStimChansAvail,1))
    # #coder['varsize('SS['AmpMin',[NumStimChansAvail,1],[1,0])
    # SS['FreqMax'] = np.zeros((NumStimChansAvail,1))
    # #coder['varsize('SS['FreqMax',[NumStimChansAvail,1],[1,0])
    # SS['FreqMin'] = np.zeros((NumStimChansAvail,1))
    # #coder['varsize('SS['FreqMin',[NumStimChansAvail,1],[1,0])
    # SS['DEKA_SensOFF'] = np.zeros((13,1)) ## 0 is ON, 1 is OFF ## MB0611
    # # SS['DEKA_SensOFF'] = np.zeros((19,1) ## MB0611
    # # #coder['varsize('SS['DEKA_SensOFF',[NumStimChansAvail,1],[1,0])
    # SS['EncodeAlg'] = np.zeros((NumStimChansAvail,1))
    # #coder['varsize('SS['EncodeAlg',[NumStimChansAvail,1],[1,0])
    # SS['SensorFudge'] = np.zeros((13,1)) # MB0611 Threshold for sensor values (0 to 1): cut out variation in the baseline
    # # SS['SensorFudge'] = np.zeros((19,1) # MB0611 Threshold for sensor values (0 to 1): cut out variation in the baseline


    # ## DEKA
    # SS['DEKASensorLabels'] = ['index_medial','index_distal','middle_distal','ring_distal','pinky_distal','palm_pinky','palm_thumb','palm_side','palm_back','thumb_ulnar','thumb_medial','thumb_distal','thumb_dorsal'] #13x1 cell of strings ##TODO: Will dimension matter later? Just using a list. 
    # SS['DEKAMotorLabels'] = ['wrist_PS','wrist_FE','thumb_pinch','thumb_yaw','index','middle',] # 6x1 cell ##TODO: Will dimension matter later? Just using a list. 
    # SS['ContDEKASensors'] = np.zeros((13,1)) #double of sensor values
    # # SS['ContDEKASensors = np.zeros((19,1) # MB0611
    # SS['PastDEKASensors'] = np.zeros((13,5)) #double buffer of sensor values
    # # SS['PastDEKASensors = np.zeros((19,5) # MB0611
    
    # SS['ContDEKAPositions'] = np.zeros((6,1))
    # SS['PastDEKAPositions'] = np.zeros((6,5)) # 5 digits, 1 extra thumb dof, 2 wrists
    # SS['DEKASensorThresholds'] = np.zeros((13,2)) # the threshold from the sensor above which stim occurs
    # # SS['DEKASensorThresholds = np.zeros((19,2) # MB0611 the threshold from the sensor above which stim occurs
    
    # SS['RH'] = 0 ## set right (1) or left hand (0)
    ################## End Mark's Originals ##################################
    
    return SS