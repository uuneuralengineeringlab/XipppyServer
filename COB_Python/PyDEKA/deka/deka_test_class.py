import can4python as can
import time
import threading


class DekaControl():

    def __init__(self):
        DEFAULT_KCD_FILE = '/usr/local/share/deka_server/deka_can.kcd'

        self.aci_msg = {'chan1_1': 0, 'chan1_2': 0, 'chan1_3': 0, 'chan1_4': 0,
            'chan2_1': 0, 'chan2_2': 0, 'chan2_3': 0, 'chan2_4': 0,
            'chan3_1': 0, 'chan3_2': 0, 'chan3_3': 0, 'chan3_4': 0,
            'chan4_1': 0, 'chan4_2': 0, 'chan4_3': 0, 'chan4_4': 0}

        self.bus = can.CanBus.from_kcd_file(DEFAULT_KCD_FILE,'can0',ego_node_ids=['1'])

        self.thread = threading.Thread(target=self.run)

    def start(self):
        self.thread.start()


    def run(self):
        self.basetime = time.time() 
        while True:
            r = self.bus.recv_next_signals()
            if 'sync' in r:
                curtime = time.time()-self.basetime
                if (curtime > 5) & (curtime <= 6):
                   self.aci_msg['chan4_1'] = 1000
                elif (curtime > 6):
                    self.aci_msg['chan4_1'] = 1000
                    if int(time.time()%10) > 5:
                        self.aci_msg['chan2_1'] = 1000
                    else:
                        self.aci_msg['chan2_1'] = 0
                self.bus.send_signals(self.aci_msg)
            elif 'sen_index_tip' in r:
                # print(r)
                pass
            
if __name__ == '__main__':
    d = DekaControl()
    d.start()