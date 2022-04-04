# -*- coding: utf-8 -*-
"""
Created on Wed Jan 13 10:33:10 2021

@author: Administrator
"""

import numpy as np

def find_bad_chans(SS):
    if SS['bad_EMG_elecs'].size > 0:
        SS['bad_EMG_chans'] = (np.any(np.transpose(SS['EMG_diff_matrix'])[SS['bad_EMG_elecs'],:], axis=0)
                               .nonzero()[0])
    else:
        SS['bad_EMG_chans'] = np.array([])
    
    return SS
        
if __name__ == '__main__':
    import feedbackdecode as fd
    SS = fd.initSS()
    