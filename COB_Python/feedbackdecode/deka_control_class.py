import can4python as can
import socket
import struct
import multiprocessing as mp
import numpy as np
import time



class DekaControl():

    def __init__(self, direct_aci=False):
        
        DEFAULT_KCD_FILE = '/usr/local/lib/python3.4/dist-packages/feedbackdecode/deka_can.kcd'
        server_addr = '192.168.42.1'

        self.aci_msg = {
            'chan1_1': 0, 'chan1_2': 0, 'chan1_3': 0, 'chan1_4': 0,
            'chan2_1': 0, 'chan2_2': 0, 'chan2_3': 0, 'chan2_4': 0,
            'chan3_1': 0, 'chan3_2': 0, 'chan3_3': 0, 'chan3_4': 0,
            'chan4_1': 1000, 'chan4_2': 0, 'chan4_3': 0, 'chan4_4': 0}
        
        
        self.XS_data = np.zeros(6) # data coming from XipppyServer
        self.sensors = np.zeros(19) # sensor values coming from LUKE
        self.WristMode = 0 #velocity=0, position=1
        # self.HandOffTS = time.time() #last time since 
        # self.GetTS = True
        self.LastACITS = time.time()
        self.sensorbaseline = np.zeros(19)
        self.handedness = 0 #right=0, left=1
        
        self.simult_delay = 1000
        self.cal_mvmt = np.hstack((
            np.linspace( 0,  0, self.simult_delay), # time to set simultaneous mode
            np.linspace( 0,  0, 200), # time to get to zero position
            np.linspace( 0,  1, 100),
            np.linspace( 1,  1, 150),
            np.linspace( 1,  0, 100),
            np.linspace( 0, -1, 100),
            np.linspace(-1, -1, 150),
            np.linspace(-1,  0, 100),
            np.linspace( 0,  0, 100)
            ))
        self.cal_iter = None
        
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
        self.rest_pos_aci = np.array([225,  400, 350,  900,    0,    0], dtype=np.int16) 
        self.min_pos_aci  = np.array([  0,    0,   0,  500, 1024, 1024], dtype=np.int16)
        self.max_pos_aci  = np.array([450, 1024, 950, 1024, 1024, 1024], dtype=np.int16)
        
        #Min/max from DEKA manual (actual values provided in min_pos_sens and max_pos_sens)
        #thumb: 0->5760
        #ind: 0->5760
        #mrp: 0->5760
        #thumbint: 0->4800
        #wristfe: -3520->3520
        #wristrot: -7680->1120
        self.min_pos_sens  = np.array([   0,    0,    0,  3000, -3520,  -7680], dtype=np.int16)
        self.max_pos_sens  = np.array([2450, 5575, 5330,  6360,  3520,  11200], dtype=np.int16)
        with np.errstate(divide='ignore', invalid='ignore'):
            self.rest_pos_sens = (self.rest_pos_aci-self.min_pos_aci) / \
                (self.max_pos_aci-self.min_pos_aci) * \
                    (self.max_pos_sens-self.min_pos_sens) + \
                        self.min_pos_sens
            self.rest_pos_sens[4:6] = 0
        
        
        if direct_aci:
            self.run_direct_aci()
        else:
            self.run()
            
            
        
    def _interp(self):
        interp_aci = np.zeros(len(self.XS_data), dtype=np.int16)
        for i in range(len(self.XS_data)):
            if self.XS_data[i] >= 0:
                interp_aci[i] = self.rest_pos_aci[i] + \
                    self.XS_data[i] * (self.max_pos_aci[i] - self.rest_pos_aci[i]) #Take the middle value -rest and find the diff with Max then add Rest
            else:                                                    #E.G. 1024 - 900 = 124 * .80 = 99.2 + 900 = 999.2 
                interp_aci[i] = self.rest_pos_aci[i] - \
                    abs(self.XS_data[i] * (self.rest_pos_aci[i] - self.min_pos_aci[i])) #Take the middle value -rest and find the abs diff with Min then add Min again
            #                                                        # E.G. 500 - 900 = abs(400 * -.80) = 900-320  = 580
        return interp_aci


    def start(self):
        self.proc.start()
        
        
    def map_deka(self):
        # 0 thumb f/e # 1 index # 2 mrp # 3 thumb add/abd # 4 wrist FE # 5 wrist Rot
        interp_aci = self._interp()
        
        self.aci_msg['chan1_4'] = np.clip(interp_aci[0],0,1024)
        self.aci_msg['chan2_1'] = np.clip(interp_aci[1],0,1024)
        self.aci_msg['chan2_3'] = np.clip(interp_aci[2],0,1024)
        self.aci_msg['chan1_1'] = np.clip(interp_aci[3],0,1024)
        
        # wrist rotation is velocity mode, have to account for it
        if self.handedness: #left
            if interp_aci[5] >= 0:
                self.aci_msg['chan3_2'] = np.clip(interp_aci[5],0,1024)
                self.aci_msg['chan3_1'] = 0
            else: 
                self.aci_msg['chan3_2'] = 0
                self.aci_msg['chan3_1'] = np.clip(abs(interp_aci[5]),0,1024)
        else: #right
            if interp_aci[5] >= 0:
                self.aci_msg['chan3_1'] = np.clip(interp_aci[5],0,1024)
                self.aci_msg['chan3_2'] = 0
            else: 
                self.aci_msg['chan3_1'] = 0
                self.aci_msg['chan3_2'] = np.clip(abs(interp_aci[5]),0,1024)
            # wrist flexion is velocity mode, have to account for it
        
        if self.handedness: #left
            if interp_aci[4] >= 0:
                self.aci_msg['chan3_4'] = np.clip(interp_aci[4],0,1024)
                self.aci_msg['chan3_3'] = 0
            else: 
                self.aci_msg['chan3_4'] = 0
                self.aci_msg['chan3_3'] = np.clip(abs(interp_aci[4]),0,1024)
        else: #right
            if interp_aci[4] >= 0:
                self.aci_msg['chan3_3'] = np.clip(interp_aci[4],0,1024)
                self.aci_msg['chan3_4'] = 0
            else: 
                self.aci_msg['chan3_3'] = 0
                self.aci_msg['chan3_4'] = np.clip(abs(interp_aci[4]),0,1024)
 
        
    def direct_aci_deka(self, kin):
        kin = np.int16(kin)
        self.aci_msg = {
            'chan1_1': kin[0], 'chan1_2': kin[1], 'chan1_3': kin[2], 'chan1_4': kin[3],
            'chan2_1': kin[4], 'chan2_2': kin[5], 'chan2_3': kin[6], 'chan2_4': kin[7],
            'chan3_1': kin[8], 'chan3_2': kin[9], 'chan3_3': kin[10], 'chan3_4': kin[11],
            'chan4_1': kin[12], 'chan4_2': kin[13], 'chan4_3': kin[14], 'chan4_4': kin[15]}
    
    
    def norm_force_sensor(self, sensor, baseline):
        if self.cal_iter is not None:
            return sensor
        else:
            return np.clip((sensor-baseline)/(255-baseline),0,1)
    
    
    def norm_pos_digit(self, sensor, ind):
        # normalize [-1, 1]
        rest = self.rest_pos_sens[ind]
        minpos = self.min_pos_sens[ind]
        maxpos = self.max_pos_sens[ind]
        if sensor >= rest:
            mapped_sensor = (sensor-rest)/abs(maxpos-rest) 
        else:
            mapped_sensor = (sensor-rest)/abs(minpos-rest) 
        
        return np.clip(mapped_sensor,-1,1)
    
    
    def norm_pos_wrist(self, sensor, ind): 
        # normalize [-1, 1]
        rest = self.rest_pos_sens[ind]
        minpos = abs(self.min_pos_sens[ind])
        maxpos = self.max_pos_sens[ind]
        
        if (ind == 4) and (self.handedness == 0): #negate wrist flexion sensor value 
            sensor = -sensor
            
        if (ind == 5) and (self.handedness == 1): #negate wrist flexion sensor value 
            sensor = -sensor
            
        if sensor >= rest:
            mapped_sensor = sensor/maxpos 
        else:
            mapped_sensor = sensor/minpos
        
        return np.clip(mapped_sensor,-1,1)
    
        
    
    def deka_pos_vel_map(self):
        # get sensor values from wrist and modify for velocity or position mode
        mapped_rot = self.sensors[13]
        mapped_flex = self.sensors[14] #for deka, negative is flexion
                
        rot_targ = self.XS_data[5]
        flex_targ = self.XS_data[4]
                                    
        if self.WristMode:
            # for position mode
            new_rotation = (rot_targ - mapped_rot)*5 # Tyler had 1, original 5
            new_rotation = np.clip(new_rotation, -1, 1)
            new_flex = (flex_targ - mapped_flex)*2 # Tyler had 0.5, original 2
            new_flex = np.clip(new_flex, -1, 1)
        else:
            # for velocity
            new_rotation = rot_targ
            new_flex = flex_targ
                        
        self.XS_data[5] = new_rotation
        self.XS_data[4] = new_flex
                    
                
    def run(self):
        while True:
            try: #Receive message from Xippy server, then respond with sensor vals
                #receive data for motor commands
                data = np.array(struct.unpack('<7f',self.udp.recv(1024)))
                self.XS_data = data[:6]
                self.WristMode = data[6]
                self.deka_pos_vel_map() # changes XS_data[4:5] for wrist position/velocity
                self.map_deka()
                
                sensor_msg = struct.pack('<19f', *self.sensors) # *self.sensors unpacks items into individual items
                
                self.udp.sendto(sensor_msg,('localhost',20003))
            except:
                data = None
                
            try: #Receiving message from LUKE arm
                aci_recvd = self.bus.recv_next_signals()
                
                if 'sync' in aci_recvd:
                    
                    #Move hand and find max sensor values
                    self.LastACITS = time.time()
                    
                    # if self.GetTS:
                        # self.GetTS = False
                        
                    if self.cal_iter is not None:
                        print('cal_iter', self.cal_iter)
                        if self.cal_iter < self.simult_delay: # let hand get into simultaneous mode
                            pass
                        else: # proceed through movements                
                            kin = self.cal_mvmt[self.cal_iter]
                            self.XS_data = np.array([kin,kin,kin,-0.5,kin,0])
                            self.WristMode = 1
                            self.deka_pos_vel_map()
                            self.map_deka()
                            self.sensorbaseline = np.maximum(self.sensorbaseline,
                                                              self.sensors)
                        
                        self.cal_iter += 1
                        if self.cal_iter == self.cal_mvmt.size:
                            print('sensor calibration complete')
                            print('self.sensorbaseline:', self.sensorbaseline)
                            self.sensorbaseline = self.sensorbaseline + 5 #nudging baseline to avoid continuous stim (range is 0 to 255 -> newtons*10)
                            self.cal_iter = None
                    
                    # print(self.aci_msg.items())
                    
                    self.bus.send_signals(self.aci_msg)
                    # print(self.sensors)
                elif 'sen_index_lat' in aci_recvd:
                    self.sensors[0] = self.norm_force_sensor(aci_recvd['sen_index_lat'],self.sensorbaseline[0])
                    self.sensors[1] = self.norm_force_sensor(aci_recvd['sen_index_tip'],self.sensorbaseline[1])
                    self.sensors[2] = self.norm_force_sensor(aci_recvd['sen_mid_tip'],self.sensorbaseline[2])
                    self.sensors[3] = self.norm_force_sensor(aci_recvd['sen_ring_tip'],self.sensorbaseline[3])
                    self.sensors[4] = self.norm_force_sensor(aci_recvd['sen_pinky_tip'],self.sensorbaseline[4])
                elif 'sen_palm_distal' in aci_recvd:
                    self.sensors[5] = self.norm_force_sensor(aci_recvd['sen_palm_distal'],self.sensorbaseline[5])
                    self.sensors[6] = self.norm_force_sensor(aci_recvd['sen_palm_prox'],self.sensorbaseline[6])
                    self.sensors[7] = self.norm_force_sensor(aci_recvd['sen_hand_edge'],self.sensorbaseline[7])
                    self.sensors[8] = self.norm_force_sensor(aci_recvd['sen_hand_dorsal'],self.sensorbaseline[8])
                elif 'sen_thumb_ulnar' in aci_recvd:
                    self.sensors[9] = self.norm_force_sensor(aci_recvd['sen_thumb_ulnar'],self.sensorbaseline[9])
                    self.sensors[10] = self.norm_force_sensor(aci_recvd['sen_thumb_rad'],self.sensorbaseline[10])
                    self.sensors[11] = self.norm_force_sensor(aci_recvd['sen_thumb_vol'],self.sensorbaseline[11])
                    self.sensors[12] = self.norm_force_sensor(aci_recvd['sen_thumb_dor'],self.sensorbaseline[12])
                elif 'wrist_pron' in aci_recvd:
                    self.sensors[13] = self.norm_pos_wrist(aci_recvd['wrist_pron'],5)
                    self.sensors[14] = self.norm_pos_wrist(aci_recvd['wrist_flex'],4)
                    self.sensors[15] = self.norm_pos_digit(aci_recvd['index_finger'],1)
                    self.sensors[16] = self.norm_pos_digit(aci_recvd['mrp'],2)
                    # self.sensors[15] = aci_recvd['index_finger']
                    # self.sensors[16] = aci_recvd['mrp']
                elif 'thumb_pitch' in aci_recvd:
                    self.sensors[17] = self.norm_pos_digit(aci_recvd['thumb_pitch'],3) #intrinsic
                    self.sensors[18] = self.norm_pos_digit(aci_recvd['thumb_yaw'],0) #flex/extend
                    # self.sensors[17] = aci_recvd['thumb_pitch']
                    # self.sensors[18] = aci_recvd['thumb_yaw']
                elif 'handedness' in aci_recvd:
                    self.handedness = aci_recvd['handedness']

            except:
                curtime = time.time()
                if curtime-self.LastACITS>5:
                    # self.GetTS = True
                    self.cal_iter = 0
                    self.aci_msg = {
                        'chan1_1': 0, 'chan1_2': 0, 'chan1_3': 0, 'chan1_4': 0,
                        'chan2_1': 0, 'chan2_2': 0, 'chan2_3': 0, 'chan2_4': 0,
                        'chan3_1': 0, 'chan3_2': 0, 'chan3_3': 0, 'chan3_4': 0,
                        'chan4_1': 1000, 'chan4_2': 0, 'chan4_3': 0, 'chan4_4': 0}
                # if self.GetTS:
                #     self.HandOffTS = curtime
                aci_recvd = None
                
        print('closing deka control')
        
        
    def run_direct_aci(self):
        while True:
            try:
                data = struct.unpack('<17f',self.udp.recv(1024))
                # print(data)
                if data[0]:
                    break
                self.direct_aci_deka(data[1:])
                # print(self.aci_msg)
            except:
                data = None
                
            try:
                aci_recvd = self.bus.recv_next_signals()
                if 'sync' in aci_recvd:
                    self.bus.send_signals(self.aci_msg)
                elif 'sen_index_tip' in aci_recvd:
                    # print(aci_recvd)
                    pass
            except:
                # print('deka control timeout')
                aci_recvd = None
                
        print('closing deka control')
      

            
if __name__ == '__main__':
    d = DekaControl()
