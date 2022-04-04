import feedbackdecode as fd
# import glob
# import multiprocessing as mp
import serial # pySerial for vibrotactile stim
import re # used to search for attached usb devices
import socket
import struct
import subprocess
# from sys import platform
import time
import xipppy as xp
import select
import numpy as np

################ Only runs on Nomad ##########################
RootDir = r'/var/rppl/storage'
ClientAddr = "192.168.42.129" 
ServerAddr = "192.168.42.1" 
ClientAddrWifi = "192.168.43.129"
ServerAddrWifi = "192.168.43.1" 
ClientAddrDEKA = 'localhost'
ServerAddrDEKA = 'localhost'


############################## Initialize XippPy #############################
while True:
    try:
        xp._open()
        time.sleep(0.5)
        pre_time = xp.time()
        time.sleep(0.5)
        if (xp.time()-pre_time)>10000:
            print('xipppy successfully connected...')
            break
        else:
            xp._close();
    except:
        print('waiting on xipppy...')
time.sleep(0.5)


############################ Initialize SS Dict ##############################
SS = fd.initSS()
SS['avail_chans'] = np.array(xp.list_elec())


######################### Create eventparams file ############################
timestr = time.strftime('%Y%m%d-%H%M%S')
SS['eventparams_fid'] = open(RootDir + r'/eventparams/eventparams' + 
                             timestr + r'.ep', 'w')


################# Initialize vibrotactile stim ###############################
try:
    usb_id = subprocess.check_output(['dmesg'])
    usb_id = re.findall('ch341-uart converter now attached to ttyUSB\d',str(usb_id))
    usb_id = re.search('ttyUSB\d',usb_id[-1])
    
    SS['VT_ard'] = serial.Serial('/dev/' + usb_id.group(0))
    SS['VT_ard'].baudrate = 250000
    print('Vibrotactile arduino connected')
    # SS['VT_ard'].close()
except:
    print('Vibrotactile arduino failed to connect...')


    
    
####### set filters for lfp (only need 1st channel of each frontend) #########
chan = int(SS['all_EMG_chans'][0])
xp.signal_set(chan, 'raw', False)#; time.sleep(0.1)
xp.signal_set(chan, 'hi-res', False)#; time.sleep(0.1)
xp.signal_set(chan, 'lfp', True)#; time.sleep(0.1)
xp.filter_set(chan, 'lfp', 3)#; time.sleep(0.1)
xp.filter_set(chan, 'lfp notch', 2)#; time.sleep(0.1)
for chan in SS['all_EMG_chans']:
    if chan in SS['avail_chans']:
        xp.signal_set(int(chan), 'spk', False)#; time.sleep(0.1) #spk must be set for each channel
    else:
        print('No EMG detected in Port D')

################# Try to turn off streams we don't need ######################
for chan in SS['neural_FE_idx']:
    if chan in SS['avail_chans']:
        xp.signal_set(chan, 'raw', False)#; time.sleep(0.1)
        xp.signal_set(chan, 'hi-res', False)#; time.sleep(0.1)
        xp.signal_set(chan, 'lfp', False)#; time.sleep(0.1)
for chan in SS['all_neural_chans']:
    if chan in SS['avail_chans']:
        xp.signal_set(int(chan), 'spk', False)#; time.sleep(0.1)
        try:
            xp.signal_set(int(chan), 'stim', True)#; time.sleep(0.1)
        except:
            print('Not a +stim front end. Chan:', chan) 
    
########################### enable stim ######################################
xp.stim_enable_set(True)#; time.sleep(0.1)
time.sleep(0.1)
if not xp.stim_enable(): # returns true if stim was enabled correctly
    print('Stim not correctly enabled in initialization')
if sum(np.in1d(np.arange(6), SS['avail_chans'])) == 0: # if we don't have electrical stim channels
    SS['avail_chans'] = np.hstack((SS['avail_chans'], np.arange(6))) # add VTstim channels


    
########## socket for continuously communicating with matlab gui #############
mat_cont_udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
mat_cont_udp.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
mat_cont_udp.bind((ServerAddr, 20001)) # listen for gui comms on 20001
# mat_cont_udp.setblocking(0) # sending to matlab through 20002

mat_cont_udp_wifi = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
mat_cont_udp_wifi.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
mat_cont_udp_wifi.bind((ServerAddrWifi, 20001)) # listen for gui comms on 20001

########## socket for event communication with matlab gui ####################
mat_evnt_udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
mat_evnt_udp.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
mat_evnt_udp.bind((ServerAddr, 20005)) # listen for gui comms on 20005
# mat_evnt_udp.setblocking(0) # sending to matlab through 20006

mat_evnt_udp_wifi = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
mat_evnt_udp_wifi.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
mat_evnt_udp_wifi.bind((ServerAddrWifi, 20005)) # listen for gui comms on 20005

########### socket for communicating with deka_test_class.py ##################
udp_deka = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_deka.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
udp_deka.bind((ClientAddrDEKA, 20003)) # listen for DEKA comms on 20003
udp_deka.setblocking(0) # sending to deka driver through 20004

ClientAddrList = [ClientAddr,ClientAddrWifi]
UDPCont = [mat_cont_udp,mat_cont_udp_wifi]
UDPEvnt = [mat_evnt_udp,mat_evnt_udp_wifi]

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
header = np.r_[np.size(SS['cur_time']), 
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


###### flush deka_server buffer to prevent any delays with transmission ######
while True:
    try:
        raw_DEKA_udp = udp_deka.recv(1024)
    except:
        break

############### Make sure xipppy is working! #################################
while True:
    try:
        xp._open()
        time.sleep(0.5)
        pre_time = xp.time()
        time.sleep(0.5)
        if (xp.time()-pre_time)>10000:
            print('xipppy successfully connected...')
            break
        else:
            xp._close();
    except:
        print('waiting on xipppy...')
time.sleep(0.5)


################################# VERY LAST ##################################
######################### Write initial SS dict ##############################
SS['cur_time'] = xp.time()#; time.sleep(0.1)
SS['eventparams_fid'].write(fd.SS_to_string(SS) + '\n')



##############################################################################
########################## Loop starts here ##################################
##############################################################################
while True:
    # begLoop = time.time()
    ############### DO NOT PLACE CODE ABOVE THIS #############################
    ####### calc curTime/preTime/elapsedTime and get new EMG #################
    SS = fd.get_features(SS) # this returns diff pairs
    
    # getFeat = time.time()
    #################### start mimicry training ##############################
    if SS['train_iter'] is not None:
        SS = fd.mimic_training(SS)
        
        
    ############## load kdf file and train kalman parameters #################
    SS = fd.load_train_Kalman(SS, RootDir, UDPEvnt, ClientAddrList) # sends event to GUI when training complete
           

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
    
    # getDecode = time.time()
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
        #print(SS['cur_sensors'][:5])
    except:
        print('unable to grab DEKA sensors')

    # getStim = time.time()
    ########################### Stim stuff ###################################
    if SS['stop_stim']:
        SS['stim_freq_save'] = np.zeros(SS['stim_freq_save'].size) # save stim freq for three USEAs (0-95, 128-223, 256-351)
        SS['stim_amp_save'] = np.zeros(SS['stim_amp_save'].size) # save stim amp for three USEAs
    else:
        SS = fd.stim_engine(SS)
        
            
    #### Save .eyn data right after unpack (13-18 are position sensors) ######
    SS['eyn_fid'].write(np.r_[SS['cur_time'], 
                              SS['feat'], 
                              SS['xhat'].flatten(), 
                              SS['cur_sensors'],
                              SS['stim_freq_save'],
                              SS['stim_amp_save']].astype('single'))

    # getComm = time.time()
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
        # print(SS['calc_time'], '276 - calc', SS['elapsed_time'], '276 - elapsed')
        pdata = struct.pack('<81f',*np.hstack((SS['elapsed_time'],
                                               SS['calc_time'],
                                               SS['feat'][SS['sel_feat_idx']],
                                               SS['kin'][:6],
                                               SS['xhat'][:6].flatten(),
                                               SS['cur_sensors'])))

    
    readable, writable, exceptional = select.select(UDPCont, UDPCont, UDPCont)
    for u in writable:
        if u is UDPCont[0]: #lan
            u.sendto(pdata,(ClientAddrList[0],20002))
        elif u is UDPCont[1]: #wifi
            u.sendto(pdata,(ClientAddrList[1],20002))
    
    
    ################### read from XipppyClientGUI ############################
    data = ['']
    readable, writable, exceptional = select.select(UDPEvnt, UDPEvnt, UDPEvnt)
    for u in readable:
        data = u.recv(1024).decode('UTF-8')
        SS['eventparams_fid'].write(data + "; SS['cur_time'] = " + 
                                    str(SS['cur_time']) + ';\n')
        data = data.split(':',1)
        print(data)
    
    
    ######################## GUI event parsing ###############################
    if data[0] == 'close': # break out of the loop
        break
    SS = fd.guiCOMM(SS, data, RootDir, UDPEvnt, ClientAddrList)
    
    

    ############### Find calculation time and sleep ##########################
    SS['calc_time'] = np.single((xp.time()-SS['cur_time'])/30)
    # endLoop = time.time()
    # print(f'Feat: {getFeat - begLoop:.4f}, Decode: {getDecode - getFeat:.4f}, Stim: {getStim - getDecode:.4f}, Comm: {getComm - getStim:.4f}, End: {endLoop - getComm:.4f}')
    # print(SS['calc_time'], '316 - calc')
    time.sleep(min(33,abs(33-SS['calc_time']))/1000)


######################### Close communications ###############################
xp._close()
mat_cont_udp.close()
mat_evnt_udp.close()
mat_cont_udp_wifi.close()
mat_evnt_udp_wifi.close()
udp_deka.close()
SS['eventparams_fid'].close()
SS['eyn_fid'].close()
if SS['VT_ard'] is not None:
    SS['VT_ard'].close()

