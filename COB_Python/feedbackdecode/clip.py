# -*- coding: utf-8 -*-
"""
Created on Tue Oct  5 14:42:07 2021

@author: Administrator
"""

def clip(val, minval, maxval):
    if val<minval:
        val = minval
    elif val>maxval:
        val = maxval
        
    return val