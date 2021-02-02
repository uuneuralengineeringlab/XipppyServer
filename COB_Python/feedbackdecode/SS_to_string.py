# -*- coding: utf-8 -*-

def SS_to_string(SS):
    '''
    Writes SS dict's keys and values in a nicely formatted string, exluding 
    certain fields in unwanted_fields (generally large numpy arrays).

    Parameters
    ----------
    SS : SS dict after initialization.

    Returns
    -------
    desired_string : string formatted nicely from SS dict, with 
        unwanted_fields removed..

    '''
    
    desired_string = []
    
    ####### KEEP unwanted_fields IN SAME ORDER AS initSS.py ##################
    unwanted_fields = ['prevTime',              ## Timing items
                       'calcTime',
                       'elapsedTime',
                       'eventparams_fid',       ## General File I/O
                       'eyn_fid',
                       'EMG_diff_matrix',       ## Decode items
                       'previousEMGbuff',
                       'feat',
                       'train_seq',
                       'kin',
                       'train_fid',
                       'xhat',
                       'xhat_raw',
                       'K',
                       'StateMod',
                       'xhat_prev',
                       'iterationBuffCounter',
                       'curSensors',            ## Encode items
                       'pastSensors',
                       'StimCmd',
                       'NextPulse',
                       'StimSeq'
                       ]
    for key, val in SS.items():
        if key not in unwanted_fields:
            txt = ('{key}: {val}'.format(key=str(key), val=str(val))
                   .replace('\n', '')) # get rid of newline characters
            desired_string.append(txt)
            
    return str(desired_string)
