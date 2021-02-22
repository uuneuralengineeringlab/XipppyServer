import can4python as can
import time
import socket
import struct
import numpy as np

class DekaTest():
    


    def __init__(self):
        
        DEFAULT_KCD_FILE = '/usr/local/lib/python3.4/dist-packages/feedbackdecode/deka_test.kcd'
        server_addr = '192.168.42.1'
        self.gui_addr = '192.168.42.129'

        self.aci_msg = {
            'chan1_1': 0, 'chan1_2': 0, 'chan1_3': 0, 'chan1_4': 0,
            'chan2_1': 0, 'chan2_2': 0, 'chan2_3': 0, 'chan2_4': 0,
            'chan3_1': 0, 'chan3_2': 0, 'chan3_3': 0, 'chan3_4': 0,
            'chan4_1': 1000, 'chan4_2': 0, 'chan4_3': 0, 'chan4_4': 0}
        
        self.bus = can.CanBus.from_kcd_file(DEFAULT_KCD_FILE,'can0',ego_node_ids=['1'],timeout=0.01)
        
        self.udp = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.udp.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        self.udp.bind((server_addr,20004))
        
        self.udp.setblocking(0)
        
        self.currenttime = 0
        self.lasttime = 0
        self.elapsedtime = 0
              
        # self.run()
        self.run_10ms()
        
        
        
    def update_aci(self, kin):
        self.aci_msg = {
            'chan1_1': kin[0], 'chan1_2': kin[1], 'chan1_3': kin[2], 'chan1_4': kin[3],
            'chan2_1': kin[4], 'chan2_2': kin[5], 'chan2_3': kin[6], 'chan2_4': kin[7],
            'chan3_1': kin[8], 'chan3_2': kin[9], 'chan3_3': kin[10], 'chan3_4': kin[11],
            'chan4_1': kin[12], 'chan4_2': kin[13], 'chan4_3': kin[14], 'chan4_4': kin[15]}
        
      
        
    def run(self):
        
        while True:
            try:
                data = struct.unpack('<17f',self.udp.recv(1024))
                if data[0]:
                    break
                self.update_aci(data[1:])
            except:
                data = None
                
            try:
                aci_recvd = self.bus.recv_next_signals()
                if 'sync' in aci_recvd:
                    self.bus.send_signals(self.aci_msg)
                    self.currenttime = time.time()
                    self.elapsedtime = self.currenttime-self.lasttime
                    self.udp.sendto(struct.pack('<1f',self.elapsedtime),(self.gui_addr,20003))
                    self.lasttime = self.currenttime
            except:
                aci_recvd = None

    def run_10ms(self):
        last_send = time.time()
        last_recv = 0
        recvd_sync = False
        while True:
            try:
                data = struct.unpack('<17f',self.udp.recv(1024))
                if data[0]:
                    break
                self.update_aci(data[1:])
            except:
                data = None
                
            try:
                aci_recvd = self.bus.recv_next_signals()
                # print('recv')
                last_recv = time.time()
                if 'sync' in aci_recvd:
                    recvd_sync = True
                    self.bus.send_signals(self.aci_msg)
                    print('SENT!!!!!')
                    last_send = time.time()
                    self.currenttime = time.time()
                    self.elapsedtime = self.currenttime-self.lasttime
                    self.udp.sendto(struct.pack('<1f',self.elapsedtime),(self.gui_addr,20003))
                    self.lasttime = self.currenttime
            except:
                print('excepted')
                aci_recvd = None
                
            if recvd_sync and last_recv > last_send and time.time() - last_send > 0.01:
                self.bus.send_signals(self.aci_msg)
                last_send = time.time()
                print('over 10ms')
                
      

            
if __name__ == '__main__':
    d = DekaTest()
