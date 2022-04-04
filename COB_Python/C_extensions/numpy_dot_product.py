# -*- coding: utf-8 -*-
"""
Created on Fri Jul  9 09:04:04 2021

@author: Administrator
"""

#python_dot_product.py

import time
import numpy as np
import random as rn

def timer(func):
    def wrapper(*args, **kwargs):
        before = time.time()
        result = func(*args, **kwargs)
        after = time.time()
        return after - before, result
    return wrapper


def generate(size, range_):
    arr1 = [[rn.randrange(*range_) for _ in range(size[1])] for _ in range(size[0])]
    arr2 = [[rn.randrange(*range_) for _ in range(size[0])] for _ in range(size[1])]
    return [np.array(arr1),np.array(arr2)]


@timer
def numpy_implementation(arr1, arr2):
    return np.dot(arr1, arr2)


if __name__ == '__main__':
    # data = generate(size=(50,500), range_=(1, 100))
    # data2 = generate(size=(50,500), range_=(1, 100))
    data = [np.random.rand(496, 32), np.random.rand(32,33)]
    # numpy_time_taken, numpy_result = numpy_implementation(*data)
    # print("time taken with numpy:", numpy_time_taken, "seconds")
    start = time.time()
    for i in range(1000):
        numpy_time_taken, numpy_result = numpy_implementation(*data)
        # numpy_time_taken, numpy_result = numpy_implementation(*data2)
        # time.sleep(.033)
    end = time.time()
    print(end-start)