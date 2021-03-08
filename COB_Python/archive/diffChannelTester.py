# -*- coding: utf-8 -*-
"""
Created on Mon Mar  8 11:27:49 2021

@author: Administrator
"""

import numpy as np

diff_mat = SS['EMG_diff_matrix']

idx1 = np.where(SS['EMG_diff_matrix'][0,:])
idx2 = np.where(SS['EMG_diff_matrix'][1,:])
idx3 = np.where(SS['EMG_diff_matrix'][2,:])
idx4 = np.where(SS['EMG_diff_matrix'][3,:])


allidx = np.unique(np.vstack((idx1, idx2, idx3, idx4)))




goodidx = SS['EMG_diff_matrix'][:4,:]

goodchans = []

for i in range(diff_mat.shape[1]):
     if np.sum(np.abs(goodidx[:,i])) == 2:
         goodchans.append(i)
         
         