'''
This is a Xippy test script that will run through the following operations:

1) Initialize Xipppy
2) Find all Micro_stim, micro, micro HV, and Nano channels (if they exist) and front ends
3) Configure the data streams on the Front Ends
4) Set filters for the front end LFP and HiRes data streams
4.5) Configure fast settle settings on each front ent
5) Create and execute a stimulation pattern for a stim FE
6) Collect Data
7) Plot all collected data including delivered stimulation
8) close xippmex

Required packages in environment: xipppy, numpy, matplotlib, math, and time


Written by Landan Mintch 10.30.2019 (Ripple Neuro, LLC)

For questions/concerns, contact: support@rppl.com

1. Package Initialization
'''

import xipppy as xp
import numpy as np
import matplotlib.pyplot as plt
import math
import time

#number of seconds of recording
n_sec=2 
    
def ms_to_clk(ms):
    clk=np.ceil(ms*300)
    return clk.astype(np.int64)
#1    
if __name__== '__main__':
    with xp.xipppy_open():
        fs_clk=3E5
        num_elec=32
#2      '''Find all the channels associated with micro and nano front ends as well as unique front ends'''
        stimChans=np.array(xp.list_elec('stim'))
        stimFEs=np.unique(np.floor(np.divide(stimChans,num_elec)))
        recChans=np.array(xp.list_elec('nano')+xp.list_elec('micro'))
        recFEs=np.unique(np.floor(np.divide(recChans,num_elec)))
        
        timeZero=xp.time()
        
#3      '''Configure electrode stim FE to have stim data streams active, all hi-rest streams to active, and deactivate all other streams'''
        if len(stimChans): 
            for jj in stimChans:
                xp.signal_set(jj.item(),'stim',True)
                xp.signal_set(jj.item(),'spk',False)
                time.sleep(0.001) 
                #pause has to be used bc property-changing fxns are asynchronous n fucky
            for kk in range(0,len(stimFEs)):
                xp.signal_set(stimChans[num_elec*kk].item(),'hi-res',True)
                xp.signal_set(stimChans[num_elec*kk].item(),'raw',False)
                #xp.signal_set(stimChans[num_elec*kk].item(),'lfp',False) #commenting out fixes huuuuge artifact in the lfp channel
                time.sleep(0.001)
        if len(recChans):
            #for num in stimChans:
                #if num in recChans:
                    #recChans.remove(num)
            for ii in recChans:
                xp.signal_set(ii.item(),'spk',False)
                time.sleep(0.001)
            for ii in range(0,8):
                xp.signal_set(ii,'spk',True) 
                time.sleep(0.001)
            for zz in range(0,len(recFEs)):
                xp.signal_set(recChans[num_elec*zz].item(),'lfp',True)
                xp.signal_set(recChans[num_elec*zz].item(),'raw',False)
                #xp.signal_set(recChans[num_elec*zz].item(),'hi-res',False) #commenting out fixes issue w/ truncate
                time.sleep(0.001)

#4          ''' set the hi-res notch filter for the first stim FE to uses the 60/120/180 comb filter'''
        if len(stimChans):
            xp.filter_set(stimChans[0].item(),'hires notch',2)
        ''' set the LFP filter for the first recording FE to use the wide-band 1.0-175 Hz filter'''
        if len(recChans):
            xp.filter_set(recChans[0].item(),'lfp',1)
#4.5    '''set the stim fast settle to 'non' for front end 0, i.e. duration = 0 ms '''
        if len(stimChans):
            xp.fast_settle(stimFEs[0].astype(np.int64).item(),'stim',0,) 
        
        time.sleep(0.5)

#5      ''' Create stim settings'''
        #timeZero=xp.time()
        if len(stimChans):
            xp.stim_enable_set(True)
            ''' create a cathodic and anodic phase for 30 clock cycles with an amplitude of 20'''
            cath_ph=xp.StimSegment(30,20,-1)
            anod_ph=xp.StimSegment(30,20,1)
            
            '''Create a stimulation pattern for electrode 5&6 on the first Stim FE with bipolar stimulation between the electrodes, with a frequency of 30 Hz and a train length of one second. In this case, electrode 4 will have the cathodic phase first and electrode 5 will have the anodic phase first. So for the first phase, current will flow from electrode 4 into electrode 5 '''
            
            seq0=xp.StimSeq(stimChans[3].item(),1000,30,cath_ph,anod_ph)
            seq1=xp.StimSeq(stimChans[4].item(),1000,30,anod_ph,cath_ph)
            
            #send out the stimulation sequences
            xp.StimSeq.send_stim_seqs([seq0,seq1])
            
            time.sleep(1.5) #still don't know why i need this
#6      ''' Collect the data'''
        n_hires=n_sec*2000 #number of hires data points (1 sec)
        n_lfp=n_sec*1000 #number of lfp data points (1 sec)
        
        if len(stimChans):
            #Collect stim count and stim data for channel 4
            stimCount3,stimData3=xp.stim_data(3)
            #Collect stim count and stim data for channel 5
            stimCount4,stimData4=xp.stim_data(4)
            #Collect hi-res data and timestamps for channels 1-3
            hiresData,hiresTimestamp=xp.cont_hires(n_hires,stimChans[0:3])

        if len(recChans):
            #Collect spike count and spike data for channel 8
            spkCount,spkData=xp.spk_data(7)
            #Collect LFP data and timestamps for channels 1-3
            lfpData,lfpTimestamp=xp.cont_lfp(n_lfp,recChans[0:3])
            
            time1=xp.time()
        
        
#7 '''Plot everything'''

# plot hires - in groups of n_hires
        if len(hiresData):
            time_hires=np.linspace(0,n_sec,n_hires)
            plt.figure(1)
            for j in range(0,np.divide(len(hiresData),n_hires).astype('int')):
                plt.plot(time_hires,hiresData[j*n_hires:j*n_hires+n_hires],label='Ch {}'.format(j))
            plt.ylabel('Recorded Voltage (uV)')
            plt.xlabel('Time (s)')
            plt.title('Hi-Res Data')
            plt.legend()
#plot spike 
        if len(spkCount):
            time_spk=range(0,52)
            plt.figure(2)
            for k in range(0,spkCount):
                plt.plot(time_spk,spkData[k].wf)
            plt.ylabel('Recorded Voltage (uV)')
            plt.xlabel('Samples (#)')
            plt.title('Spike Waveforms for Ch 8')

# plot lfp - in groups of n_lfp
        if len(lfpData):
            plt.figure(3)
            time_lfp=np.linspace(0,n_sec,n_lfp)
            for i in range(0,np.divide(len(lfpData),n_lfp).astype('int')):
                plt.plot(time_lfp,lfpData[i*n_lfp:i*n_lfp+n_lfp],label='Ch {}'.format(i))
            plt.ylabel('Recorded Voltage (uV)')
            plt.xlabel('Time (s)')
            plt.title('LFP Data')
            plt.legend()
        plt.show()
        