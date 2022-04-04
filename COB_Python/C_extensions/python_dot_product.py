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
    return [arr1,arr2]


@timer
def python_implementation(arr1, arr2):
    result = [[0 for _ in range(len(arr1))] for _ in range(len(arr2[0]))]
    for i in range(len(arr1)):
        for j in range(len(arr2[0])):
            for k in range(len(arr2)):
                result[i][j] += arr1[i][k] * arr2[k][j]
    return result

@timer
def numpy_implementation(arr1, arr2):
    return np.array(arr1).dot(arr2)

@timer
def numpy2_implementation(arr1, arr2):
    arr1 = np.array(arr1)
    arr2 = np.array(arr2)
    result = np.zeros((np.size(arr1,0),np.size(arr2,1)))
    for i in range(np.size(arr1,0)):
        for j in range(np.size(arr2,1)):
            for k in range(np.size(arr2,0)):
                result[i][j] += arr1[i][k] * arr2[k][j]
    return result


if __name__ == '__main__':
    data = generate(size=(50,500), range_=(1, 100))
    numpy_time_taken, numpy_result = numpy_implementation(*data)
    numpy2_time_taken, numpy2_result = numpy2_implementation(*data)
    python_time_taken, python_result = python_implementation(*data)
    print(f"time taken with numpy: {numpy_time_taken} seconds")
    print(f"time taken with numpy2: {numpy2_time_taken} seconds")
    print(f"time taken with python: {python_time_taken} seconds")