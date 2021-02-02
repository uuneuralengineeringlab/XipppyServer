# -*- coding: utf-8 -*-
"""
Created on Mon Apr  6 13:33:02 2020

@author: Administrator
"""

import xipppy as xp
import numpy as np
#import matplotlib.pyplot as plt
import time
import socket
import struct
import feedbackdecode as fd
import glob
import os

udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp.bind(("192.168.42.1", 20001))
udp.setblocking(0)

try:
    xp._open()
except:
    time.sleep(0.5)
    xp._open()
    
# xp.signal_set(0, 'lfp', True)

numChans = 32
bufLen = 300
curTime = 0.0
prevTime = 0.0
elapsedTime = 0.0
calcTime = 0.0
chanList = np.arange(256,256+numChans) # port C for now - just acquire EMG
mLFP = np.zeros((1,numChans), dtype=np.single) # do we want single?
# start_training = 0
train_iter = None
train_seq = np.zeros((7,33*5))
kin = np.zeros(7)
train_fid = None

# kalman test parameters
xhat = np.zeros((7,1))
K = np.zeros((7,numChans))
StateMod = np.zeros((7,7))
klim = np.r_[np.zeros((7,1)), np.ones((7,1))]
z = np.zeros((numChans,1))

# kalman training parameters
kf_start_train = None



while True:
    # find times and calculate time since last loop finished
    curTime = np.float64(xp.time())
    elapsedTime = (curTime-prevTime)/30
    prevTime = curTime
    
    # get new EMG #TODO: make sure this function works properly
    mLFP = fd.get_features(bufLen, chanList, 0, numChans)
    
    # d = xp.cont_lfp(bufLen, chanList, 0)
    # d = np.transpose(np.reshape(d[0],(numChans,bufLen)))    
    # mLFP = np.single(np.mean(np.abs(d),axis=0))
    
    # start mimicry training #TODO: Make this a function that just returns train_iter -- And kf_start_train, yes?
    if train_iter is not None:
        train_iter, kf_start_train = fd.mimic_train(kin, train_iter, train_seq, train_fid, curTime, mLFP, kf_start_train)
        # kin[:] = train_seq[:, train_iter]
        # train_fid.write(np.r_[curTime, mLFP, kin].astype('single'))
        # if train_iter == train_seq.shape[1]-1: # finished training
        #     train_iter = None
        #     train_fid.close()
        #     print('Training finished')
        #     kf_start_train = 1
        # else:
        #     train_iter += 1
            
    # start KF training #TODO: Make this a function
    if kf_start_train is not None:
        # everything below probably should be parallelized
        
        # load most recent training file
        import pdb; pdb.set_trace()
        # list_of_files = glob.glob('/srv/data/training_KDFs/*.kdf')
        list_of_files = glob.glob(r'C:\Users\Administrator\Code\COB\COB_Python\training_KDFs/*.kdf')
        latest_file = max(list_of_files, key=os.path.getctime)
        train_fid = open(latest_file, 'br', buffering=0)
        train_contents = np.fromfile(train_fid, dtype='single')
        train_fid.close()
        
        # parse file
        header = train_contents[:3].astype('int')
        train_data = train_contents[3:]
        train_data = train_data.reshape(-1,sum(header))
        idxs = np.cumsum(header)
        NIP_times = train_data[:,:idxs[0]]
        features = train_data[:,idxs[0]:idxs[1]]
        kinematics = train_data[:,idxs[1]:idxs[2]]
        
        # train KF then get steady state
        (A, W, _, _, _, H, Q, _) = fd.kf_train_cob(kinematics, features)
        K, StateMod, _ = fd.kf_getss_cob(A, W, H, Q)

    
    # xhat = fd.kf_test_cob(xhat, z, K, StateMod, klim)
        
    
    
    # send to XipppyClientGUI
    # s = "SS="+np.array2string(mLFP)+";"
    # udp.sendto(str.encode(s),("127.0.0.1",20002))
    # udp.sendto(mLFP,("192.168.42.131",20002))
    pdata = struct.pack('<34f',*np.hstack((elapsedTime,calcTime,mLFP)))
    udp.sendto(pdata,("192.168.42.131",20002))
    try:
        data = udp.recv(1024).decode('UTF-8')
        # import pdb; pdb.set_trace()
        # data.decode('UTF-8')
        print(data)
    except:
        data = ''
        
    # below would be a helpful function as well
    if data == 'close':
        break
    elif data == 'StartTraining':
        # start_training = True
        train_iter = 0
        timestr = time.strftime('%Y%m%d-%H%M%S')
        train_fid = open('/srv/data/training_KDFs/trainKDF_' + timestr + '.kdf', 'wb') # nomad directory
        # train_fid = open(r'C:\Users\Administrator\Code\COB\COB_Python\training_KDFs/trainKDF_' + timestr + '.kdf', 'wb') # windows directory for debugging
        # write header
        header = np.r_[curTime.size, mLFP.size, kin.size].astype('single')
        # np.save(train_fid, header, allow_pickle=False)
        train_fid.write(header.astype('single'))
        
        
    
    calcTime = (np.float64(xp.time())-curTime)/30
    
    time.sleep(min(33,abs(33-calcTime))/1000)



xp._close()
udp.close()


