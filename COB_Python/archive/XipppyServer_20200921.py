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

udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp.bind(("192.168.42.1", 20001))
udp.setblocking(0)

try:
    xp._open('tcp')
except:
    time.sleep(1)
    xp._open('tcp')
    
# xp.signal_set(0, 'lfp', True)

numChans = 32
dataPoints = 37
dataLen = 0
bufLen = 300
curTime = 0.0
prevTime = 0.0
elapsedTime = 0.0
calcTime = 0.0
prevtic = 0.0
bufIdx = np.arange(bufLen)
chanList = np.arange(256,256+numChans) # port C for now - just acquire EMG
lfpBuff = np.zeros((bufLen,numChans))
mLFP = np.zeros((1,numChans), dtype=np.single)


while True:
    curTime = np.float(xp.time())
    # d = xp.cont_lfp(maxLen, chanList, int(curTime-1110))
    d = xp.cont_lfp(datapoin, chanList, 0)
    # print(d[0][0])
    d = np.transpose(np.reshape(d[0],(numChans,maxLen)))
    # print(d[0,0:3])
    # mLFP = np.single(np.mean(np.abs(d),axis=0))
    #print(mLFP)
    
    elapsedTime = (curTime-prevTime)/30
    prevTime = curTime
    # print(elapsedTime)
    dataLen = int(elapsedTime)
    if dataLen>maxLen:
        dataLen = maxLen
    
    dataIdx = np.arange(dataLen)
    d = d[dataIdx,:] #trim to match elapsed time
    
    lfpBuff[bufIdx[dataIdx],:] = d
    bufIdx = np.roll(bufIdx,dataLen)
    
    mLFP = np.single(np.mean(np.abs(lfpBuff),axis=0))
    
    # s = "SS="+np.array2string(mLFP)+";"
    # udp.sendto(str.encode(s),("127.0.0.1",20002))
    # udp.sendto(mLFP,("192.168.42.131",20002))
    pdata = struct.pack('<34f',*np.hstack((elapsedTime,calcTime,mLFP)))
    udp.sendto(pdata,("192.168.42.131",20002))
    try:
        data = udp.recv(1)
    except:
        data = b'\x00'
    if (data==b'\x01'):
        break
    
    calcTime = (np.float(xp.time())-curTime)/30
    # print([calcTime,elapsedTime])
    
    time.sleep(min(33,abs(33-calcTime))/1000)



xp._close()
udp.close()
