import can4python as can
import socket
import struct
import multiprocessing as mp
import numpy as np


class DekaControl():

    def __init__(self, direct_aci=False):
        DEFAULT_KCD_FILE = '/usr/local/share/deka_server/deka_can.kcd'
        # DEFAULT_KCD_FILE = r"C:\Users\Administrator\Code\COB\COB_Python\PyDEKA\deka\deka_can.kcd"
        server_addr = '192.168.42.1'

        self.aci_msg = {
            'chan1_1': 0, 'chan1_2': 0, 'chan1_3': 0, 'chan1_4': 0,
            'chan2_1': 0, 'chan2_2': 0, 'chan2_3': 0, 'chan2_4': 0,
            'chan3_1': 0, 'chan3_2': 0, 'chan3_3': 0, 'chan3_4': 0,
            'chan4_1': 1000, 'chan4_2': 0, 'chan4_3': 0, 'chan4_4': 0}

        self.bus = can.CanBus.from_kcd_file(DEFAULT_KCD_FILE,'can0',ego_node_ids=['1'],timeout=0.01)

        self.udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.udp.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        if direct_aci:
            self.proc = mp.Process(target=self.run_direct_aci)
            self.udp.bind((server_addr, 20004))
        else:
            self.proc = mp.Process(target=self.run)
            self.udp.bind(('localhost', 20004))
        
        self.udp.setblocking(0)
        
        # below are values for interpolating kinematics to aci messages
        # thumb, ind, mrp, thumbint, wrist fe, wrist rot
        self.rest_pos_aci = np.array([225, 400, 350, 900, 0, 0], dtype=np.int16) 
        self.min_pos_aci = np.array([0, 0, 0, 500, 0, 0], dtype=np.int16)
        self.max_pos_aci = np.array([450, 1024, 950, 1024, 1024, 1024], dtype=np.int16)
        
    ## TODO: finish these interpolations. Need to figure out wrist mapping
    # def _interp(self, newpos):
    #     interp_aci = np.empty(len(newpos), dtype=int)
    #     for i in range(len(newpos)):
    #         if newpos[i] >= 0:
    #             interp_aci[i] = self.rest_pos_aci + \ 
    #                 newpos * (self.max_pos_aci - self.rest_pos_aci) #Take the middle value- rest and find the diff with Max then add Rest
    #         else:                                                    #E.G. 1024 - 900 = 124 * .80 = 99.2 + 900 = 999.2 
    #             interp_aci[i] = self.min_pos_aci + \
    #                 newpos * (self.rest_pos_aci - self.min_pos_aci) #Take the middle value- rest and find the abs diff with Min then add Min again
                                                                      #E.G. 500 - 900 = 400 * .80 = 320 + 500 = 820
    #     return interp_aci

    def start(self):
        self.proc.start()
        
    def map_deka(self,data):
        # 0 thumb add/abd # 1 index # 2 mrp # 3 thumb FE # 4 wrist FE # 5 wrist Rot
        # I think 0 being thumb f/e and 3 thumb add/abd is better
        data_mapped = (data[1]+1)*500
        return data_mapped
        
        ##TODO: finish below to have interpolation and set aci messages separately
        # interp_aci = self._interp(data)
        # self.aci_msg['chan1_4'] = 
        # self.aci_msg['']
    
    def direct_aci_deka(self, kin):
        kin = np.int16(kin)
        self.aci_msg = {
            'chan1_1': kin[0], 'chan1_2': kin[1], 'chan1_3': kin[2], 'chan1_4': kin[3],
            'chan2_1': kin[4], 'chan2_2': kin[5], 'chan2_3': kin[6], 'chan2_4': kin[7],
            'chan3_1': kin[8], 'chan3_2': kin[9], 'chan3_3': kin[10], 'chan3_4': kin[11],
            'chan4_1': kin[12], 'chan4_2': kin[13], 'chan4_3': kin[14], 'chan4_4': kin[15]}
        
    # def set_rest(self, positions):
    #     self.rest_position = positions
    
        
    def run(self):
        while True:
            try:
                data = struct.unpack('<6f',self.udp.recv(1024))
                # print(data)
                if data[0]:
                    break
                self.aci_msg['chan2_1'] = self.map_deka(data)
                # self.map_deka(data)
            except:
                data = None
                
            try:
                r = self.bus.recv_next_signals()
                if 'sync' in r:
                    self.bus.send_signals(self.aci_msg)
                    # self.bus.send_signals({'chan4_1': 1000})
                elif 'sen_index_tip' in r:
                    # print(r)
                    pass
            except:
                # print('deka control timeout')
                r = None
                
        print('closing deka control')
        
    def run_direct_aci(self):
        while True:
            try:
                data = struct.unpack('<17f',self.udp.recv(1024))
                print(data)
                if data[0]:
                    break
                self.direct_aci_deka(data[1:])
                print(self.aci_msg)
            except:
                data = None
                
            try:
                r = self.bus.recv_next_signals()
                if 'sync' in r:
                    self.bus.send_signals(self.aci_msg)
                elif 'sen_index_tip' in r:
                    # print(r)
                    pass
            except:
                # print('deka control timeout')
                r = None
                
        print('closing deka control')
      

            
if __name__ == '__main__':
    d = DekaControl()
    d.start()