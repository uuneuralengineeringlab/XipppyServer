# -*- coding: utf-8 -*-

import socket
import struct
import time

def XS_close(server_addr='192.168.42.1', server_port=20001):
    
    # create our socket
    kill_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    kill_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    kill_socket.bind((server_addr, 20111)) # port that this would hypothetically listen to
    
    # format our message
    pack_fmt = '<6s'
    msg = bytes('close:', 'utf-8')
    pack_msg = struct.pack(pack_fmt, msg)
    
    # send the message to kill XipppyServer
    kill_socket.sendto(pack_msg, (server_addr, server_port))
    
    # sleep for a sec to let XS shut down properly
    time.sleep(1)
    
if __name__ == '__main__':
    XS_close()
    