# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 14:12:53 2021

@author: TNT
"""

def tie_DOFs(SS):
    for i in range(len(SS['mirror_DOF'])):
        if SS['mirror_DOF'][i] != 0: #if number isn't 0
            SS['xhat'][i] = SS['xhat'][SS['mirror_DOF'][i]-1] ##sub 1 since we use 0 as do nothing
    return SS