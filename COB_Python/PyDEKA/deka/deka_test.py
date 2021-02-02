import can4python as can
import time

DEFAULT_KCD_FILE = '/usr/local/share/deka_server/deka_can.kcd'

aci_msg = {'chan1_1': 0, 'chan1_2': 0, 'chan1_3': 0, 'chan1_4': 0,
           'chan2_1': 0, 'chan2_2': 0, 'chan2_3': 0, 'chan2_4': 0,
           'chan3_1': 0, 'chan3_2': 0, 'chan3_3': 0, 'chan3_4': 0,
           'chan4_1': 0, 'chan4_2': 0, 'chan4_3': 0, 'chan4_4': 0}

bus = can.CanBus.from_kcd_file(DEFAULT_KCD_FILE,'can0',ego_node_ids=['1'])

basetime = time.time()                                      

while True:
    r = bus.recv_next_signals()
    if 'sync' in r:
        curtime = time.time()-basetime
        if (curtime > 5) & (curtime <= 6):
           aci_msg['chan4_1'] = 1000
        elif (curtime > 6):
            aci_msg['chan4_1'] = 1000
            if int(time.time()%10) > 5:
                aci_msg['chan1_2'] = 1000
            else:
                aci_msg['chan1_2'] = 0
        bus.send_signals(aci_msg)
    elif 'sen_index_tip' in r:
        print(r)
