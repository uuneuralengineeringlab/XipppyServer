# -*- coding: utf-8 -*-
"""
Created on Thu Sep 10 10:26:45 2020

@author: TNT
"""
import numpy as np

def  readWTSfile(filedir): #codegen # was readWTSfile(SS)
    ## Find the most recent .WTS file (Write Training Sequence file)
    # c_fmt = ['ls -t /usr/UofU/WTS/*.WTS | head -1 >/usr/UofU/WTS/MostRecent.log', char(0)];
    
    # if SS['ToLoadWTS']: # there is a specific file (SS.KTFfile) to load
    #     ms_log('Loading new WTS')
    #     filename = SS.WTSfile;
    #     SS.ToLoadWTS = false();
    #     writeMostRecentfile('/usr/UofU/WTS/',filename) #write to MostRecent for default restart
    # else
    #     # read MostRecent.log
    #     ms_log('Reading most recent WTS')
    #     path = '/usr/UofU/WTS/MostRecent.log'
    #     filename = ReadFilename(path);
    #     if isempty(filename) # if empty then find most recent and write to log
    #         c_fmt = ['ls -1iqXr /usr/UofU/WTS/TrainingSeq/*.WTS | cut -d " " -f 2| head -1>/usr/UofU/WTS/MostRecent.log',char(0)]; # use inode to sort and write most recent to .txt file
    # #         c_fmt = ['ls -1iqXr /usr/UofU/KTF/TrainedKF/*.KTF | cut -d " " -f 2| head -1>/usr/UofU/KTF/MostRecent.log']; # use inode to sort and write most recent to .txt file
    #         coder.ceval('system',c_fmt); # executes the c_fmt command in the linux operating system
    #         filename = ReadFilename(path); # reread the log to get the most recent
        
    # 
    # if SS.WTSfile(1:7) == 'Default'
    #     c_fmt = ['ls -1iqXr /usr/UofU/WTS/TrainingSeq/*.WTS | cut -d " " -f 2| head -1>/usr/UofU/WTS/MostRecent.log',char(0)]; # use inode to sort
    #     coder.ceval('system',c_fmt);
    #     logname = '/usr/UofU/WTS/MostRecent.log';
    #     fid = fopen(logname,'r');
    #     readdata = fread(fid);
    #     fclose(fid);
    #     filename = char(readdata(1:end-1))';
    # else
    #     filename = SS.WTSfile;
    # 
    
    
    
    # FullFilename = r'C:\Users\Administrator\Code\COB\COB_DevFolder\TrainingSequenceFiles\5DOF_10Trials.WTS'
    # ms_log('Reading wts file...')
    # ms_log(FullFilename)
    
    ## Load WTS data
    fid = open(filedir, 'br', buffering=0)
    data1 = np.fromfile(fid, dtype='single')
    rdDate = data1[0:6]
    RdArraySize = data1[6:10]
    RdArrayLength = data1[10:13]
    fid.close()
    idx = np.cumsum(RdArrayLength);
    idx = idx +12;
    ## Reshape linear array into original dimensions
    KalmanMat = np.reshape(data1[int(idx[0]):int(idx[1])], (int(RdArraySize[0]),int(RdArraySize[1])), order='F')
    DEKAMat = np.reshape(data1[int(idx[1]):int(idx[2])], (int(RdArraySize[2]),int(RdArraySize[3])), order='F')

    #ms_log('DEKAMat loaded')
    
    ## for debugging check
    # ms_open()
    # ms_log('KMat Size1: #f',size(KalmanMat,1));
    # ms_log('KMat Size2: #f',size(KalmanMat,2));
    # mvntIdx = 1;
    # ms_deka_open();
    # ms_deka_mode();
    # xl_pause(1);
    # while mvntIdx < size(KalmanMat,2)
    # #     ms_log('KMat: #f', sum(KalmanMat(:,mvntIdx)))
    #     xl_pause(0.033);
    #     posDEKA = Xhat2DEKA ([DEKAMat(1,mvntIdx),DEKAMat(2,mvntIdx),DEKAMat(3,mvntIdx),DEKAMat(4,mvntIdx),DEKAMat(5,mvntIdx),DEKAMat(6,mvntIdx)]);
    #     deka_move_COB(posDEKA);
    #     ms_log('MvntIdx: #f',mvntIdx)
    #     mvntIdx = mvntIdx+1;
    # 
    # ms_close()
    
    return KalmanMat,DEKAMat