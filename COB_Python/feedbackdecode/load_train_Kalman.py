# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 14:21:45 2021

@author: Administrator
"""

import feedbackdecode as fd
import numpy as np
import multiprocessing as mp
import struct

def load_train_Kalman(SS, RootDir, mat_evnt_udp=None, ClientAddr=None):
    if SS['train_kf_phase'] is not None:
        if SS['train_kf_phase'] == 'StartChanSel':
            # grab data from .kdf
            SS, kin, feat = fd.readKDFFile(SS, RootDir)
            SS['train_kin'] = kin
            SS['train_feat'] = feat
            # find channels to be excluded
            SS = fd.find_bad_chans(SS)
            # set up channel selection on another process
            movements = np.arange(6)
            ######### below enables debugging channel selection ###########
            #### can place a debugger inside gramSchmDarpa as well ########
            # import pdb; pdb.set_trace()
            # SS['sel_feat_idx'] = fd.gramSchmDarpa(SS['train_kin'].T, 
            #                                   SS['train_feat'].T, 
            #                                   movements, 
            #                                   SS['num_features'], 
            #                                   SS['chan_sel_queue'],
            #                                   SS['bad_EMG_chans'])
            ######### end debugging section ################
            ctx = mp.get_context('spawn') # type of new process
            SS['chan_sel_queue'] = mp.Queue() # enables communications
            SS['chan_sel_proc'] = mp.Process(target=fd.gramSchmDarpa, 
                                             args=(SS['train_kin'].T, 
                                                   SS['train_feat'].T, 
                                                   movements, 
                                                   SS['num_features'], 
                                                   SS['chan_sel_queue'],
                                                   SS['bad_EMG_chans']))
            SS['chan_sel_proc'].start()
            print('Started channel selection on another core')
            SS['train_kf_phase'] = 'WaitChanSel'
            SS['stop_hand'] = 1
            
        elif SS['train_kf_phase'] == 'WaitChanSel':   
            if not SS['chan_sel_proc'].is_alive():
                SS['sel_feat_idx'] = SS['chan_sel_queue'].get()
                SS['chan_sel_queue'].close()
                
                SS['chan_sel_proc'].join()
                print('channels selected')
                print(SS['sel_feat_idx'])
                SS['train_kf_phase'] = 'TrainKF'
                
        elif SS['train_kf_phase'] == 'TrainKF':
            # This could also be sent to another thread if slow
            # train KF then get steady state
            (A, W, _, _, _, H, Q, _) = fd.kf_train_cob(SS['train_kin'], SS['train_feat'][:,SS['sel_feat_idx']])
            print('Kalman Trained')
            SS['K'], SS['StateMod'], _ = fd.kf_getss_cob(A, W, H, Q)
            print('SS Kalman Trained')
            fd.save_decode_params(SS, RootDir)
            SS['train_kf_phase'] = None
            print('Decode parameters saved')
            SS['stop_hand'] = 0
            
            # send to GUI so it populates fields correctly
            stop_hand_cmd = 'TrainingFinished:stop_hand = ' + str(SS['stop_hand'])
            udpstr = bytes(stop_hand_cmd + ';', 'utf-8')
            pack_fmt = '<' + str(len(udpstr)) + 's'
            pdata = struct.pack(pack_fmt, udpstr)
            mat_evnt_udp.sendto(pdata, (ClientAddr,20006))


            
        else:
            print('Should not be here (fd.load_train_kalman)')
    
    return SS
