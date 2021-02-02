# -*- coding: utf-8 -*-
"""
Created on Thu Aug 13 12:00:40 2020

@author: Administrator
"""
from itertools import combinations
import numpy as np
def nchoosek(n,k):
    
    return np.array(list(combinations(np.arange(0,n),k)))