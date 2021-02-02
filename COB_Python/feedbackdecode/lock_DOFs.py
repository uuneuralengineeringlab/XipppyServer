# -*- coding: utf-8 -*-

import numpy as np

def lock_DOFs(SS):
    '''
    Locks SS['xhat'] based on SS['LockDOF'] bools to their respective 
    SS['kin']

    Parameters
    ----------
    SS : SS dict from fd.initSS()

    Returns
    -------
    SS : SS dict with modified SS['xhat']

    '''
    
    if np.any(SS['LockDOF']):
        SS['xhat'][SS['LockDOF'],0] = SS['kin'][SS['LockDOF']]
        
    if np.any(SS['LockDOF'][4:6]):
        SS['WristMode'] = 1 # position wrist
    else:
        SS['WristMode'] = 0 # velocity wrist
    
    return SS