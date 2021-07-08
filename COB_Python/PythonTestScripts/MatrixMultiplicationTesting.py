# -*- coding: utf-8 -*-
"""
Created on Thu Jul  8 14:34:11 2021

@author: Administrator
"""
import numpy as np
import time

emg = np.random.random((496, 32))
d = np.random.random((32,33))

def dot(emg, d, iters=10000):
    tic = time.time()
    for _ in range(iters):
        ans = np.dot(emg, d)
    return (time.time() - tic) / iters


def matmul(emg, d, iters=10000):
    # ans = np.zeros((496, 33), dtype=np.float32)
    tic = time.time()
    for _ in range(iters):
        ans = sgemm(1, emg, d)
    return (time.time() - tic) / iters
# out=variable doesn't change much
# float32 is about twice as fast as float64
# casting='no' is no faster
# order='C' is no faster
# numpy.matmul with scipy.linalg.blas.sgemm(...) is faster for 64, not for 32
# changing order 'F' or 'C', some improvement, but not significant..

def at(emg, d, iters=10000):
    tic = time.time()
    for _ in range(iters):
        ans = emg @ d
    return (time.time() - tic) / iters


dott = dot(emg, d)
print(f'dot: {dott}')


matmult = dot(emg, d)
print(f'matmul: {matmult}')
        