# -*- coding: utf-8 -*-

import numpy as np

def lock_DOFs(SS):
    '''
    Locks SS['xhat'] based on SS['lock_DOF'] bools to their respective 
    SS['kin']

    Parameters
    ----------
    SS : SS dict from fd.initSS()

    Returns
    -------
    SS : SS dict with modified SS['xhat']

    '''
    
    if np.any(SS['lock_DOF']):
        SS['xhat'][SS['lock_DOF'],0] = SS['kin'][SS['lock_DOF']]
        
    if np.any(SS['lock_DOF'][4:6]):
        SS['wrist_mode'] = 1 # position wrist
    else:
        SS['wrist_mode'] = 0 # velocity wrist
    
    return SS