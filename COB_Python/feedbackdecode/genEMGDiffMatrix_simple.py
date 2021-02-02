# -*- coding: utf-8 -*-
"""
Created on Wed Oct 14 11:42:25 2020
 This function creates a list of differential pairs
@author: TNT
"""

import numpy as np
import feedbackdecode as fd

def  genEMGDiffMatrix_simple(numChan): ##codegen
    #numChan = SS['numChansEMG'].shape[0]
    logical_positions = fd.nchoosek(numChan,2) #use the number of emg electrods, always choose 2 for pairs
    emgMatrix = np.zeros((numChan,logical_positions.shape[0])) #create empty matrix
    for i in range(0,logical_positions.shape[0]):
       emgMatrix[logical_positions[i][0],i] = 1
       emgMatrix[logical_positions[i][1],i] = -1
    
    return emgMatrix