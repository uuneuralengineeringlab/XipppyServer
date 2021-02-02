# -*- coding: utf-8 -*-
import numpy as np
import xipppy as xp
import feedbackdecode as fd


def stim_engine(SS):
    SS['active_stim'] = SS['StimParams'][np.logical_and(SS['StimParams'][:,7] == 1,SS['StimParams'][:,8] == 1),0:7]
    # SS['StimSeq'] = [] # list of Ripple's StimSeq class (for each chan)
    SS['StimIdx'] = np.zeros(SS['active_stim'].shape[0], dtype=bool) # which chan to stim on this iteration
    
    # start_time1 = time.time()
    # elapsed_time1 = 100
    # elapsed_time2 = 100
    for i in range(SS['active_stim'].shape[0]):
        # start_time = time.time()
        StimChan = SS['active_stim'][i,0]
        
        # SS['StimSeq'].append(copy.deepcopy(SS['StimCmd']))

        # set stim values
        CSF, CSA = fd.DEKA2StimCOB(SS,i)
        CSF = np.clip(CSF, 0, 300) ##TODO: Confirm FDA limit on frequency
        CSA = np.clip(CSA, 0, 100) ##TODO: Confirm FDA limit on amplitude
        # elapsed_time1 = time.time() - start_time
        
        # start_time = time.time()    
        if CSF > 0: # stimulate
            #calculating number of NIP cycles between current time and next pulse
            NextPulseDiff = np.max([np.floor(SS['NextPulse'][StimChan] - SS['curTime']),1])
            if NextPulseDiff<np.floor(0.033 * 30000): # if we need to stim before next loop would start
                SS['StimSeq'][i].electrode = int(StimChan)
                SS['StimSeq'][i].period = int(np.floor(30000/CSF))
                SS['StimSeq'][i].repeats = int(np.ceil(0.033 * CSF))
                if NextPulseDiff == 1 and CSF < (1/0.033):
                    # print('immed')
                    SS['StimSeq'][i].action = 0 # 'immed'
                else:
                    # print('curcyc')
                    SS['StimSeq'][i].action = 0 # 'curcyc'=1 #TODO: keep this 0 for now until production xipppy is released
                # SS['StimSeq'][i].segments[0].length = # fixed at 200 us
                SS['StimSeq'][i].segments[0].amplitude = int(CSA)
                # SS['StimSeq'][i].segments[2].length = # fixed at 200 us
                SS['StimSeq'][i].segments[2].amplitude = int(CSA)
                SS['NextPulse'][StimChan] = SS['curTime'] + NextPulseDiff + np.floor(30000/CSF) 
                SS['StimIdx'][i] = True
        # elapsed_time2 = time.time() - start_time
                
                
    # # elapsed_time1 = time.time() - start_time
    # if(elapsed_time1 > elapsed_time2):
    #     print('first loop takes longer than second loop')
    # else:
    #     print('second loop takes the longest!')
    # print(str(elapsed_time1) + " vs " + str(elapsed_time2))
    
    if np.any(SS['StimIdx']):
        try: # could really clean up this try
            true_seqs = []
            for i in range(len(SS['StimIdx'])):
                if SS['StimIdx'][i] == True:
                    true_seqs.append(SS['StimSeq'][i])
            xp.StimSeq.send_stim_seqs(true_seqs)
        except:
            print('unable to send stim in stim_engine.py')
        
        
    # elapsed_time_overall = time.time() - start_time1
    # print(elapsed_time_overall)
    return SS
