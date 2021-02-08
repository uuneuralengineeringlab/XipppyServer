# -*- coding: utf-8 -*-
"""
Created on Wed Oct 28 08:32:34 2020

@author: Administrator
"""

import multiprocessing as mp
import time
import numpy as np
import feedbackdecode as fd

# def chanselect(child_conn):
#     time.sleep(5)
#     print('chanselect done!')
#     child_conn.send(np.arange(48))
#     child_conn.close()


def chanselect(q, data, data2, data3, data4):
    time.sleep(2)
    q.put(np.arange(48))
    q.put(data.shape)
    q.put(data2.shape)
    q.put(data3.shape)
    q.put(data4.shape)
    # q.put(SS)
    
if __name__ == '__main__':
    RootDir = r'C:\Users\Administrator\Code\COB\COB_Python'

    ctx = mp.get_context('spawn')
    # parent_conn, child_conn = mp.Pipe() # could also use Queue() to enable multiple producers and consumers
    q = mp.Queue() 
    # p = mp.Process(target=chanselect, args=(child_conn,))
    SS = fd.initSS()
    SS, kinematics, features = fd.readKDFFile(SS, RootDir)
    movements = np.arange(6)
    # SS = {}
    # data = np.zeros((7,30000))
    # data2 = np.zeros((192, 300000))
    # data3 = np.zeros((192, 300000))
    # data4 = np.zeros((192, 300000))
    p = mp.Process(target=fd.gramSchmDarpa, args=(kinematics.T, features.T, movements, SS['num_features'], q))
    # p = mp.Process(target=chanselect, args=(q, data, SS))
    print('starting')
    p.start()
    print('started')
    for i in range(120):
        print(i)
        if p.is_alive():
            print('still alive')
        else:
            print('dead!')
            # chans = parent_conn.recv()
            chans = q.get()
            print(chans)
            # datashape = q.get()
            # data2shape = q.get()
            # data3shape = q.get()
            # data4shape = q.get()
            # newSS = q.get()
            q.close()
            # print(chans)
            # print(datashape)
            # print(data2shape)
            # print(data3shape)
            # print(data4shape)
            # print(newSS.keys())
            p.join()
            break
        time.sleep(1)
    # p.close()