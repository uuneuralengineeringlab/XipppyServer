import xipppy as xp
import numpy as np
import time
import socket
import struct
import feedbackdecode as fd
import glob
import os
import multiprocessing as mp

# ClientAddr = "192.168.42.131"
# ServerAddr = "192.168.42.1"

ClientAddr = "localhost"
ServerAddr = "localhost"

RootDir = r'C:\Users\Administrator\Code\COB\COB_Python'
# RootDir = r'/srv/data'

SS = fd.initSS()

try:
    xp._open('tcp')
except:
    time.sleep(0.5)
    xp._open('tcp')
    
    
# set filters for lfp (only need 1st channel of each frontend)
chan = int(SS['chanListEMG'][0])
xp.signal_set(chan, 'raw', False)
xp.signal_set(chan, 'hi-res', False)
xp.signal_set(chan, 'lfp', True)
xp.filter_set(chan, 'lfp', 3)
xp.filter_set(chan, 'lfp notch', 2)
for chan in SS['chanListEMG']:
    xp.signal_set(int(chan), 'spk', False)
    

# # socket for communicating with matlab gui
# udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# udp.bind((ServerAddr, 20001))
# udp.setblocking(0)

# # socket for communicating with deka_test_class.py
# udp_deka = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# udp_deka.bind(('localhost', 20003))
# udp_deka.setblocking(0)

    
# d = fd.DekaControl()
# d.start()

# for training Kalman and doing channel selection
kf_train_proc = mp.Process
    


while True:
    # find times and calculate time since last loop finished
    SS['curTime'] = np.float64(xp.time())
    SS['elapsedTime'] = (SS['curTime']-SS['prevTime'])/30
    SS['prevTime'] = SS['curTime']
    
    # get new EMG 
    SS['mLFP'] = fd.get_features(SS) # this just gets single ended features
    SS = fd.gen_all_LFP(SS) # generating all diff pairs
    
    ##TODO: add in differential pairs
    
    # start mimicry training #TODO: Make this a function that just returns SS['train_iter']
    if SS['train_iter'] is not None:
        SS = fd.mimic_training(SS)
        
            
    # load kdf file and train kalman parameters #TODO: Make this a function
    if SS['kf_load_train'] is not None:
        
        # load most recent training file
        # import pdb; pdb.set_trace()
        # list_of_files = glob.glob('/srv/data/training_KDFs/*.kdf')
        list_of_files = glob.glob(RootDir + r'/training_KDFs/*.kdf')
        latest_file = max(list_of_files, key=os.path.getctime)
        # print(f'Training on {latest_file}')
        SS['train_fid'] = open(latest_file, 'br', buffering=0)
        train_contents = np.fromfile(SS['train_fid'], dtype='single')
        SS['train_fid'].close()
        
        # parse file
        header = train_contents[:3].astype('int')
        train_data = train_contents[3:]
        train_data = train_data.reshape(-1,sum(header))
        idxs = np.cumsum(header)
        NIP_times = train_data[:,:idxs[0]]
        features = train_data[:,idxs[0]:idxs[1]]
        kinematics = train_data[:,idxs[1]:idxs[2]]
        
        ## TODO: Implement channel selection at this point, preferably on another core
        
        
        # train KF then get steady state
        (A, W, _, _, _, H, Q, _) = fd.kf_train_cob(kinematics, features)
        print('Kalman Trained')

        SS['K'], SS['StateMod'], _ = fd.kf_getss_cob(A, W, H, Q)
        print('SS Kalman Trained')
        
        SS['kf_load_train'] = None

    # kinematic prediction
    SS['xhat'] = fd.kf_test_cob(SS['xhat'], SS['mLFP'], SS['K'], SS['StateMod'], SS['klim'])
    
    # latch filter
    SS['xhat'], SS['xhat_prev'] = fd.latching_filter(SS['xhat'], SS['xhat_prev'])
        
    
    
    # send to XipppyClientGUI
    # s = "SS="+np.array2string(SS['mLFP'])+";"
    # udp.sendto(str.encode(s),("127.0.0.1",20002))
    # udp.sendto(SS['mLFP'],("192.168.42.131",20002))
    pdata = struct.pack('<48f',*np.hstack((SS['elapsedTime'],SS['calcTime'],SS['mLFP'],SS['kin'],SS['xhat'].flatten())))
    udp.sendto(pdata,(ClientAddr,20002))
    try:
        data = udp.recv(1024).decode('UTF-8')
        # import pdb; pdb.set_trace()
        # data.decode('UTF-8')
        print(data)
    except:
        data = ''
        
        
    # send to deka_control_class.py
    if data == 'close':
        pdata_deka = struct.pack('<8f',*np.hstack((1,SS['xhat'].flatten())))
    else: 
        pdata_deka = struct.pack('<8f',*np.hstack((0,SS['xhat'].flatten())))
    udp_deka.sendto(pdata_deka,('localhost',20004))
    try:
        data_deka = udp_deka.recv(1024).decode('UTF-8')
        print(data_deka)
    except:
        data_deka = ''
        
       
        
    # below would be a helpful function as well
    if data == 'close':
        break
    elif data == 'StartTraining':
        # start_training = True
        SS['train_iter'] = 0
        timestr = time.strftime('%Y%m%d-%H%M%S')
        SS['train_fid'] = open(RootDir + r'/training_KDFs/trainKDF_' + timestr + r'.kdf', 'wb') # nomad directory
        # SS['train_fid'] = open(r'C:\Users\Administrator\Code\COB\COB_Python\training_KDFs/trainKDF_' + timestr + '.kdf', 'wb') # windows directory for debugging
        # write header
        header = np.r_[SS['curTime'].size, SS['mLFP'].size, SS['kin'].size].astype('single')
        # np.save(SS['train_fid'], header, allow_pickle=False)
        SS['train_fid'].write(header.astype('single'))
    elif data =='LoadRecentWTS':
        # import pdb; pdb.set_trace()
        list_of_files = glob.glob(RootDir + r'/WTS/*.wts')
        latest_file = max(list_of_files, key=os.path.getctime)
        SS['train_seq'], _ = fd.readWTSfile(latest_file)
    elif data == 'LoadTraining':
        SS['kf_load_train'] = 1
    
        
        
        
    
    SS['calcTime'] = (np.float64(xp.time())-SS['curTime'])/30
    
    time.sleep(min(33,abs(33-SS['calcTime']))/1000)



xp._close()
udp.close()
udp_deka.close()


