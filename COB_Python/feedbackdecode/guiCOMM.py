# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 13:52:03 2021

@author: TNT
"""
import feedbackdecode as fd
import glob
import os
import time
import numpy as np
import struct
import subprocess
from sys import platform


def guiCOMM(SS, data, RootDir,mat_evnt_udp, ClientAddr): 
    if data[0] == 'StartTraining':
        # load most recent WTS
        list_of_files = glob.glob(RootDir + r'/WTS/*.wts')
        if list_of_files:
            latest_file = max(list_of_files, key=os.path.getctime)
            SS['train_seq'], _ = fd.readWTSfile(latest_file)
        else:
            print('No WTS file available.')
        
        # get file saving and train_iter started
        SS['train_iter'] = 0
        timestr = time.strftime('%Y%m%d-%H%M%S')
        
        if SS['num_EMG_chans'] == 16:
            SS['train_fid'] = open(RootDir + r'/training_KDFs/trainKDF_' + timestr + r'.kdf', 'wb') # nomad directory
            
        elif SS['num_EMG_chans'] == 32:
            SS['train_fid'] = open(RootDir + r'/training_KDFs32/trainKDF_' + timestr + r'.kdf', 'wb') # nomad directory
        
        # write header
        header = np.r_[SS['cur_time'].size, SS['feat'].size, SS['kin'].size].astype('single')
        SS['train_fid'].write(header.astype('single'))
        
        
    elif data[0] == 'LoadTraining':
        SS = fd.load_decode_params(SS, RootDir)
        
        
    elif data[0] == 'UpdateDOF':
        exec(data[1])
             
        
    elif data[0] == 'UpdateStimParams':
        exec(data[1])
        timestr = time.strftime('%Y%m%d-%H%M%S')
        stimparamFID = open(RootDir + r'/stimparams/stimparams_' + timestr + r'.sp', 'wb') # nomad directory
        # Write header containg shapes of data to be saved
        header = np.array(SS['stim_params'].shape).astype('single')
        stimparamFID.write(header.astype('single'))
        stimparamFID.write(SS['stim_params'].flatten().astype('single'))
        stimparamFID.close()
    
               
    elif data[0] == 'UpdateUserStimParams':
        exec(data[1]) # updates StimParams from user stim table (only amplitudes and on/off)
        # SS['stim_params'][np.ix_(SS['stim_params'][:,7]==1, [3,4,8])] = user_stim[:,2:5]
        timestr = time.strftime('%Y%m%d-%H%M%S')
        stimparamFID = open(RootDir + r'/stimparams/stimparams_' + timestr + r'.sp', 'wb') # nomad directory
        # Write header containg shapes of data to be saved
        header = np.array(SS['stim_params'].shape).astype('single')
        stimparamFID.write(header.astype('single'))
        stimparamFID.write(SS['stim_params'].flatten().astype('single'))
        stimparamFID.close()
        
        
    elif data[0] == 'CalibrateStim':
        exec(data[1]) # updates SS['stop_hand'] toggle
    
    
    elif data[0] == 'GetStimParams':
        cmdstr = 'StimParams: data = '
        arraystr = (np.array_repr(SS['stim_params'])
                    .replace(',\n', ';')
                    .replace(' ', '')
                    .replace(',dtype=float32','')
                    .replace(')','')
                    .replace('array(',''))
        udpstr = bytes(cmdstr + arraystr + ';', 'utf-8')
        pack_fmt = '<' + str(len(udpstr)) + 's'
        pdata = struct.pack(pack_fmt, udpstr)
        
        mat_evnt_udp.sendto(pdata, (ClientAddr,20006))
        
        
    elif data[0] == 'GetUsrStimParams': 
        cmdstr = 'UserStimParams: data = '
        arraystr = (np.array_repr(SS['stim_params'][np.ix_(SS['stim_params'][:,7] == 1, [0,1,3,4,8])])
                    .replace(',\n', ';')
                    .replace(' ', '')
                    .replace(',dtype=float32','')
                    .replace(')','')
                    .replace('array(',''))
        udpstr = bytes(cmdstr + arraystr + ';', 'utf-8')
        pack_fmt = '<' + str(len(udpstr)) + 's'
        pdata = struct.pack(pack_fmt, udpstr)
        
        mat_evnt_udp.sendto(pdata, (ClientAddr,20006))
        
        
    elif data[0] == 'UpdateBadElecs':
        exec(data[1]) # updates SS['bad_EMG_elecs'] as np array
        timestr = time.strftime('%Y%m%d-%H%M%S')
        bad_elec_FID = open(RootDir + r'/bad_elecs/bad_elecs_' + timestr + r'.elec', 'wb') # nomad directory
        # Write header containg shapes of data to be saved
        header = np.array(SS['bad_EMG_elecs'].shape).astype('single')
        bad_elec_FID.write(header.astype('single'))
        bad_elec_FID.write(SS['bad_EMG_elecs'].flatten().astype('single'))
        bad_elec_FID.close()
        
        
    elif data[0] == 'GetBadElecs': 
        cmdstr = 'BadElecs: data = '
        arraystr = (np.array_repr(SS['bad_EMG_elecs'])
                    .replace(',\n', ';')
                    .replace(' ', '')
                    .replace(',dtype=int32','')
                    .replace(')','')
                    .replace('array(',''))
        udpstr = bytes(cmdstr + arraystr + ';', 'utf-8')
        pack_fmt = '<' + str(len(udpstr)) + 's'
        pdata = struct.pack(pack_fmt, udpstr)
        
        mat_evnt_udp.sendto(pdata, (ClientAddr,20006))

        
    elif data[0] == 'TimeUpdate':
        if platform != 'win32':
            cur_time = ':'.join(data[1:]) # joins back minutes and seconds
            subprocess.check_call(['date', '-s', cur_time])
            print('Nomad time updated to:', cur_time)
            
    return SS
            