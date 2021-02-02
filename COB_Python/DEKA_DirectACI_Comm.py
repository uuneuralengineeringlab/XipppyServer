import socket
import numpy as np
import struct
"""
To use this script, run feedbackdecode.DekaControl(direct_aci=True) in python on the nomad

Adjust modules and send commands to get real-time updates to DEKA

Shut down DekaControl() server and udp when finished (from this script)
"""

ClientAddr = '192.168.42.129' #PNILabview
ServerAddr = '192.168.42.1' #Nomad

# socket for communicating with deka_test_class.py
udp_deka = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
udp_deka.bind((ClientAddr, 20003)) # receives on 20003, sends on 20004
udp_deka.setblocking(0)

kin = np.zeros(16, dtype=np.single) # actual mvnt, (kcd file description), ROM
kin[12] = 1000 # must be on to communicate

# Module 1 (chan1_X)
kin[0] = 900 # thumb intrinsic (thumb pitch up) 500: out, 1024: in, rest: 900
kin[1] = 0 # no movement (thumb pitch down)
kin[2] = 0 # no movement (thumb yaw right)
kin[3] = 225 # thumb flex (thumb yaw left) 0: extend, 450: flex, rest: 225

# Module 2 (chan2_X)
kin[4] = 406 # index flex (index flex) 0: extend, 1024: flex, rest: 400
kin[5] = 0 # no movement (thumb extend)
kin[6] = 350 # MRP flex (MRP flex) 0: extend, 950: flex, rest: 350
kin[7] = 0 # no movement (MRP extend)

# Module 3 (chan3_X)
kin[8] = 0 # wrist pronate: vel (wrist pronate) 0: no mvmt, 1024: fast pronate
kin[9] = 0 # wrist supinate: vel (wrist supinate) 0: no mvmt, 1024: fast supinate
kin[10] = 1024 # wrist flex: vel (wrist flexor extension) 0: no mvmt, 1024: fast flex
kin[11] = 0 # wrist ext: vel (wrist flexor flexion) 0: no mvmt, 1024: fast flex

# Module 4 (chan4_X)
kin[12] = 1000 # (mode selection) 
kin[13] = 0 # no movement
kin[14] = 0 # no movement
kin[15] = 0 # no movement


# send command
pdata_deka = struct.pack('<17f',*np.hstack((0,kin)))
udp_deka.sendto(pdata_deka,(ServerAddr,20004))


# shut down DekaControl() server
pdata_deka = struct.pack('<17f',*np.hstack((1,kin)))
udp_deka.sendto(pdata_deka,(ServerAddr,20004))
udp_deka.close()


try:
    data_deka = udp_deka.recv(1024).decode('UTF-8')
    print(data_deka)
except:
    data_deka = ''
    
