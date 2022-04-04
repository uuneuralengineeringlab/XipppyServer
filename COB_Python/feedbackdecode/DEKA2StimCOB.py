# -*- coding: utf-8 -*-
"""
Created on Wed Aug 12 12:06:32 2020

@author: Administrator
"""
import numpy as np

def   DEKA2StimCOB(SS,k):
    # Converts DEKA Sensor Values into Stimulation Frequency using various
    # encoding algorithms: Modified from DEKA2Stim on 5/1/2019 by MRB
    
    # SENSOR INDEXS ARE DEFINED AS:
    # SS.DEKASensorLabels = {'index_medial';'index_distal';'middle_distal';'ring_distal';...
    #     'pinky_distal';'palm_pinky';'palm_thumb';'palm_side';'palm_back';...
    #     'thumb_ulnar';'thumb_medial';'thumb_distal';'thumbdorsal'};
    # 
    # SS.DEKAMotorLabels =
    # {'wrist_PRO';'wrist_FLEX';'thumb_MCP';'thumb_ABD';'index_MCP';'middle_MCP';};
    #    
    #
    # StimCell = Region, EncodeAlg (ReceptorType in FBDecode), ?, MinAmp, MaxAmp, MinFreq, MaxFreq
    # StimCell = Electrode, DEKASensor/Mvnt Region, EncodeAlg, MinAmp, MaxAmp,MinFreq, MaxFreq
    ## Inclusion of DEKA Thresholds

    # Sensory
    # DEKASensors[0] = r['sen_index_lat']
    # DEKASensors[1] = r['sen_index_tip']
    # DEKASensors[2] = r['sen_mid_tip']
    # DEKASensors[3] = r['sen_ring_tip']
    # DEKASensors[4] = r['sen_pinky_tip']
    # DEKASensors[5] = r['sen_palm_distal']
    # DEKASensors[6] = r['sen_palm_prox']
    # DEKASensors[7] = r['sen_hand_edge']
    # DEKASensors[8] = r['sen_hand_dorsal']
    # DEKASensors[9] = r['sen_thumb_ulnar']
    # DEKASensors[10] = r['sen_thumb_rad']
    # DEKASensors[11] = r['sen_thumb_vol']
    # DEKASensors[12] = r['sen_thumb_dor']
    
    # Motor
    # DEKASensors[13] = r['wrist_pron']
    # DEKASensors[14] = r['wrist_flex']
    # DEKASensors[15] = r['index_finger']
    # DEKASensors[16] = r['mrp']
    # DEKASensors[17] = r['thumb_pitch']
    # DEKASensors[18] = r['thumb_yaw']
                                                                                        
    DEKASensors = SS['cur_sensors']
    PastDEKASensors = SS['past_sensors']
    
    ## temporary normalization
    DEKASensors = np.array(DEKASensors)
    PastDEKASensors = np.array(PastDEKASensors)
    
                                                                                       
    ## Stim Settings (StimParams is size of active stimulation channels - doesn't include all available channels only selected)
    # StimChan = SS['stim_params'][k,0] #nomad channel (0-96, 128)
    DEKARegion = SS['active_stim'][k,1] #index into DEKASensors
    EncodeAlg = SS['active_stim'][k,2] #to determine encoding algorithm
    MinAmp = SS['active_stim'][k,3]
    MaxAmp = SS['active_stim'][k,4]
    MinFreq = SS['active_stim'][k,5]
    MaxFreq = SS['active_stim'][k,6]
    
    Amp = 0
    Freq = 0
    
    c0 = DEKASensors[DEKARegion]
    c1 = PastDEKASensors[DEKARegion,0]
    c2 = PastDEKASensors[DEKARegion,1]
    c3 = PastDEKASensors[DEKARegion,2]
    # c4 = PastDEKASensors[DEKARegion,3]
    # c5 = PastDEKASensors[DEKARegion,4]
    
    dcdt0 = c0 - c1
    dcdt1 = c1 - c2
    dcdt2 = c2 - c3
    # dcdt3 = c3 - c4
    # dcdt4 = c4 - c5
    
    # original code but might be wrong!
    # dc2dt0 = c0 - c2
    # dc2dt1 = c1 - c3
    
    dc2dt0 = dcdt0-dcdt1
    dc2dt1 = dcdt1-dcdt2
    
    
    if(c0 > 0):
        if EncodeAlg == 0:
            #'bio freq'
            ## University of Chicago Biofidelic Model: Physiological frequency encoding with no amplitude encoding
            scale = 3;  #0-3 (Univ Chicago trained on 3mm indentation)
            posTerm = 557.9706 * (c0*scale) - 554.7820 * (c1*scale)   #calculate position term
            velTerm = (1559.4952 * abs(dcdt0*scale)) - (359.7767 * abs(dcdt1*scale)) - (109.1068 * abs(dcdt2*scale))   #calculate velocity term
            accTerm = (364.3545 * abs(dc2dt0*scale)) + (169.9743 * abs(dc2dt1*scale))  #calculate acceleration term
            intercept = -3.1163
            Freq = posTerm + velTerm + accTerm + intercept   #determine firing rate (Chicago model, without zero-mean guassian noise)
            # Freq[Freq < 0] = 0; #no negative freq
            Amp = MinAmp
        elif EncodeAlg == 1: #'bio amp'
            scale = 3  #0-3 (Univ Chicago trained on 3mm indentation)
            posTerm = 181.7049 * (c0*scale) - 164.2083 * (c1*scale)   #calculate position term
            velTerm = (527.9869 * abs(dcdt0*scale)) + (292.2716 * abs(dcdt1*scale))   #calculate velocity term
            accTerm = (6.3115 * abs(dc2dt0*scale)) + (21.9826 * abs(dc2dt1*scale))  #calculate acceleration term
            intercept = -2.0783
            Amp = posTerm + velTerm + accTerm + intercept   #determine population active (Chicago model, without zero-mean guassian noise)
            # Amp[Amp < 0] = 0; #no negative amp
            Freq = MinFreq
        elif EncodeAlg == 2: #'RA1'
            ## rapidly adapting type 1: stim with onset and offset only (not same as in Science Robotics but this is what FeedbackDecode (DEKA2Stim lists as RA1)
            Val = (dcdt0 + dcdt1) * 10 #constant factor to increase response
            Val = abs(Val)             #direction doesn't matter
            if(Val<0.01):
                Val = 0
            # Val[Val < .01] = 0;       #remove noise
            Freq = Val*(MaxFreq-MinFreq) + MinFreq
            Amp = Val*(MaxAmp-MinAmp) + MinAmp
        elif EncodeAlg == 3: #'scaled'
            ## scaled linear fit between min and max values
            # Val = c0*2;
            Val = c0
            Freq = Val*(MaxFreq-MinFreq) + MinFreq
            Amp = Val*(MaxAmp-MinAmp) + MinAmp
        elif EncodeAlg == 4: #'min'
            Freq = MinFreq;
            Amp = MinAmp;
        elif EncodeAlg == 5: # Vibrotactile
            Val = c0
            Freq = Val*(MaxFreq-MinFreq) + MinFreq
            Amp = 0
            Freq = 255 if Freq > 255 else Freq # cap at 255
        elif EncodeAlg == 6: #'SA1'
            ## slowly adapting type 1: stim with steadystate and onset
            Val = (dcdt0 + dcdt1) * 10 #constant factor to increase response
            Val = abs(Val)             #direction doesn't matter
            if(Val<0.01):
                Val = 0
            Val += c0
            Freq = Val*(MaxFreq-MinFreq) + MinFreq
            Amp = Val*(MaxAmp-MinAmp) + MinAmp


        
        
       
    # Limit Amp and Freq to stay within hard limits
    if(Amp < 0):
        Amp = 0;
    
    if(Freq < 0):
        Freq = 0;
    
    if(Amp > MaxAmp):
        Amp = MaxAmp;
    
    if(Freq > MaxFreq):
        Freq = MaxFreq;
    
    
    
    return (int(Freq), Amp)