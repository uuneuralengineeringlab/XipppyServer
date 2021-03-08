import feedbackdecode as fd
# import glob
# import multiprocessing as mp
import numpy as np
# import os
import socket
import struct
# import subprocess
from sys import platform
import time
import xipppy as xp


################ Set up platform-specific variables ##########################
if platform == 'win32':
    ClientAddr = "localhost"
    ServerAddr = "localhost"
    RootDir = r'C:\Users\Administrator\Box\CNI\COB\XipppyServer\COB_Python'
    # RootDir = r'C:\Users\Administrator\Code\COB\COB_Python'
    # Note: DEKA comm still not set up on DekaControl() side for Windows
    # ClientAddrDEKA = "192.168.42.131" # PNILabview
    ClientAddrDEKA = "192.168.42.129" # PNILabview (new HD)
    ServerAddrDEKA = "192.168.42.1" # Nomad
else:
    # ClientAddr = "192.168.42.129" # PNILabview
    # ClientAddr = "192.168.43.133" # PNILabview wifi
    # ClientAddr = "192.168.42.131" # PNILabview (old hard drive)
    ClientAddr = "192.168.43.132" # Tablet wifi
    # ServerAddr = "192.168.42.1" # Nomad
    ServerAddr = "192.168.43.1" # Nomad wifi
    RootDir = r'/srv/data'
    ClientAddrDEKA = 'localhost'
    ServerAddrDEKA = 'localhost'


############################ Initialize SS Dict ##############################
SS = fd.initSS()


######################### Create eventparams file ############################
timestr = time.strftime('%Y%m%d-%H%M%S')
SS['eventparams_fid'] = open(RootDir + r'/eventparams/eventparams' + 
                             timestr + r'.ep', 'w')


############################## Initialize XippPy #############################
try:
    xp._open()
except:
    time.sleep(0.5)
    xp._open()
    
    
####### set filters for lfp (only need 1st channel of each frontend) #########
chan = int(SS['all_EMG_chans'][0])
xp.signal_set(chan, 'raw', False)
xp.signal_set(chan, 'hi-res', False)
xp.signal_set(chan, 'lfp', True)
xp.filter_set(chan, 'lfp', 3)
xp.filter_set(chan, 'lfp notch', 2)
for chan in SS['all_EMG_chans']:
    xp.signal_set(int(chan), 'spk', False)
    
    
########################### enable stim ######################################
xp.stim_enable_set(True)
time.sleep(0.1)
if not xp.stim_enable(): # returns true if stim was enabled correctly
    print('Stim not correctly enabled in initialization')

    
########## socket for continuously communicating with matlab gui #############
mat_cont_udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
mat_cont_udp.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
mat_cont_udp.bind((ServerAddr, 20001)) # listen for gui comms on 20001
mat_cont_udp.setblocking(0) # sending to matlab through 20002

########## socket for event communication with matlab gui ####################
mat_evnt_udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
mat_evnt_udp.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
mat_evnt_udp.bind((ServerAddr, 20005)) # listen for gui comms on 20005
mat_evnt_udp.setblocking(0) # sending to matlab through 20006

########### socket for communicating with deka_test_class.py ##################
udp_deka = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_deka.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
udp_deka.bind((ClientAddrDEKA, 20003)) # listen for DEKA comms on 20003
udp_deka.setblocking(0) # sending to deka driver through 20004


###### flush deka_server buffer to prevent any delays with transmission ######
while True:
    try:
        raw_DEKA_udp = udp_deka.recv(1024)
    except:
        break


##### send zero values to hand to reset it to rest position (takes ~2sec) ####
for i in range(60):
    pdata_deka = struct.pack('<7f',*np.hstack((SS['kin'][:6].flatten(),1))) # last term is velocity (0) or position (1) wrist 
    udp_deka.sendto(pdata_deka,(ServerAddrDEKA,20004))
    time.sleep(0.033)
    raw_DEKA_udp = udp_deka.recv(1024)


############# Initialize continuous features saving .EYN #####################
# Always saving Everything You Need!
if SS['num_EMG_chans'] == 16:
    SS['eyn_fid'] = open(RootDir + r'/cont_EYNs/cont_EYNs_' + timestr + r'.eyn', 'wb') # nomad directory
    
elif SS['num_EMG_chans'] == 32:
    SS['eyn_fid'] = open(RootDir + r'/cont_EYNs32/cont_EYNs32_' + timestr + r'.eyn', 'wb') # nomad directory
# Write header containg shapes of data to be saved
header = np.r_[SS['cur_time'].size, 
               SS['feat'].size, 
               SS['xhat'].size, 
               SS['cur_sensors'].size,
               SS['stim_freq_save'].size,
               SS['stim_amp_save'].size].astype('single')
# Write header to top of file
SS['eyn_fid'].write(header.astype('single'))


################## Load most recent decode if available ######################
SS = fd.load_decode_params(SS, RootDir)


############### Load most recent stim params if available ####################
SS = fd.load_stim_params(SS, RootDir)


############# Load most recent bad electrodes if available ###################
SS = fd.load_bad_elecs(SS, RootDir)


########### Load most recent decode overrides (locked DOFs, etc) #############
SS = fd.load_decode_overrides(SS, RootDir)


################################# VERY LAST ##################################
######################### Write initial SS dict ##############################
SS['cur_time'] = np.float64(xp.time())
SS['eventparams_fid'].write(fd.SS_to_string(SS) + '\n')


##############################################################################
########################## Loop starts here ##################################
##############################################################################
while True:
    ############### DO NOT PLACE CODE ABOVE THIS #############################
    ####### calc curTime/preTime/elapsedTime and get new EMG #################
    SS = fd.get_features(SS) # this returns diff pairs
    
    
    #################### start mimicry training ##############################
    if SS['train_iter'] is not None:
        SS = fd.mimic_training(SS)
        
        
    ############## load kdf file and train kalman parameters #################
    SS = fd.load_train_Kalman(SS, RootDir, mat_evnt_udp, ClientAddr) # sends event to GUI when training complete
           

    ######################### Kalman prediction ##############################
    SS = fd.kf_test_cob(SS) # modifies xhat_raw and should not be changed hereafter
    
    
    ############################ Threshold ###################################
    SS = fd.decode_threshold(SS) # modifies xhat

    
    ########################### latch filter #################################
    SS = fd.latching_filter(SS) # modifies xhat and xhat_prev  
    
    
    ############################ locked DOFs #################################
    SS = fd.lock_DOFs(SS) # modifies xhat
    
    
    ######################## Tie DOFs together ###############################
    SS = fd.tie_DOFs(SS) # modified xhat
    

    ############# send to/receive from DekaControl() from deka_control_class.py ###########
    if SS['train_iter'] is not None: # if doing mimic training, send kinematics to deka
        pdata_deka = struct.pack('<7f',*np.hstack((SS['kin'][:6].flatten(),1))) # last term is velocity (0) or position (1) wrist 
    else: # send decode values
        if SS['stop_hand']:
            pdata_deka = struct.pack('<7f',*np.hstack((np.zeros(6).flatten(),1)))    
            # print("sending zeros")
        else:
            pdata_deka = struct.pack('<7f',*np.hstack((SS['xhat'][:6].flatten(),SS['wrist_mode'])))
            # print("sending xhat")
    udp_deka.sendto(pdata_deka,(ServerAddrDEKA,20004))
    
    
    SS['past_sensors'][:,1:5] = SS['past_sensors'][:,0:4]
    SS['past_sensors'][:,0] = SS['cur_sensors']
    try: #unpack sensor values from Deka since we just asked for a message
        SS['cur_sensors'] = struct.unpack('<19f',udp_deka.recv(1024)) #unpack returns a immutable tuple (you cannot write to this!)
    except:
        print('unable to grab DEKA sensors')


    ########################### Stim stuff ###################################
    if SS['stop_stim']:
        SS['stim_freq_save'] = np.zeros(96*2) # save stim freq for two USEAs (0-95, 128-223)
        SS['stim_amp_save'] = np.zeros(96*2) # save stim amp for two USEAs
    else:
        SS = fd.stim_engine(SS)
        
            
    #### Save .eyn data right after unpack (13-18 are position sensors) ######
    SS['eyn_fid'].write(np.r_[SS['cur_time'], 
                              SS['feat'], 
                              SS['xhat'].flatten(), 
                              SS['cur_sensors'],
                              SS['stim_freq_save'],
                              SS['stim_amp_save']].astype('single'))

    
    #################### send to XipppyClientGUI #############################
    # if there are loads of bad channels, pad cont data with zeros
    if SS['sel_feat_idx'].size < SS['num_features']:
        zeropad = SS['num_features'] - SS['sel_feat_idx'].size
        pdata = struct.pack('<81f',*np.hstack((SS['elapsed_time'],
                                               SS['calc_time'],
                                               SS['feat'][SS['sel_feat_idx']],
                                               np.zeros(zeropad),
                                               SS['kin'][:6],
                                               SS['xhat'][:6].flatten(),
                                               SS['cur_sensors'])))
    else:
        pdata = struct.pack('<81f',*np.hstack((SS['elapsed_time'],
                                               SS['calc_time'],
                                               SS['feat'][SS['sel_feat_idx']],
                                               SS['kin'][:6],
                                               SS['xhat'][:6].flatten(),
                                               SS['cur_sensors'])))

    mat_cont_udp.sendto(pdata,(ClientAddr,20002))
    
    
    ################### read from XipppyClientGUI ############################
    try:
        data = mat_cont_udp.recv(1024).decode('UTF-8')
        SS['eventparams_fid'].write(data + "; SS['cur_time'] = " + 
                                    str(SS['cur_time']) + ';\n')
        data = data.split(':',1)
        print(data)
    except:
        data = ['']
    
    ######################## GUI event parsing ###############################
    if data[0] == 'close': # break out of the loop
        break
    SS = fd.guiCOMM(SS, data, RootDir, mat_evnt_udp, ClientAddr)

        
    ############### Find calculation time and sleep ##########################
    SS['calc_time'] = (np.float64(xp.time())-SS['cur_time'])/30
    
    time.sleep(min(33,abs(33-SS['calc_time']))/1000)


######################### Close communications ###############################
xp._close()
mat_cont_udp.close()
mat_evnt_udp.close()
udp_deka.close()
SS['eventparams_fid'].close()
SS['eyn_fid'].close()

