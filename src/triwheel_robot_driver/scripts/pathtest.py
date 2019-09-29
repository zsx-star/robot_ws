import socket
import sys
import can 
import time
import os

os.system('sudo ip link set can0 type can bitrate 500000')
os.system('sudo ifconfig can0 up')
can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')# socketcan_native
msg = can.Message(arbitration_id=0x121, data=[10, 12, 15, 19, 24, 30, 37, 1], extended_id=False)
can0.send(msg)
time.sleep(0.1)

X_SPEED = 40
Y_SPEED = 40
Z_SPEED = 10
def mov_x(di):
    if di == 1:
        di = di << 2
        
    msg = can.Message(arbitration_id=0x121, data=[1, 0, X_SPEED, 0, 0, 0, 0, di], extended_id=False)
    can0.send(msg)
    
def mov_y(di):
    if di == 1:
        di = di << 1
        
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, Y_SPEED, 0, 0, di], extended_id=False)
    can0.send(msg)
    
def mov_z(di):
    if di == 1:
        di = di 
        
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, 0, 0, Z_SPEED, di], extended_id=False)
    can0.send(msg)
    
def mov_stop():
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
    can0.send(msg)
    time.sleep(0.1)
    
def mov_path():
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, Y_SPEED, 0, 0, 0], extended_id=False)
    can0.send(msg)
    time.sleep(10)
    
    msg = can.Message(arbitration_id=0x121, data=[1, 0, X_SPEED, 0, 0, 0, 0, 0], extended_id=False)
    can0.send(msg)
    time.sleep(10)
    
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, Y_SPEED, 0, 0, 0], extended_id=False)
    can0.send(msg)
    time.sleep(4)
    
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
server_address = ('169.254.52.228', 10000)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)
# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    print >>sys.stderr, 'waiting for a connection'
    connection, client_address = sock.accept()
    
    try:
        print >>sys.stderr, 'connection from', client_address

        # Receive the data in small chunks and retransmit it
        while True:
            data = connection.recv(16)
            print >>sys.stderr, 'received "%s"' % data

            if 'x' in data:
                if 'forward' in data:
                    mov_x(0)
                else:
                    mov_x(1)
                        
            elif 'y' in data:
                if 'forward' in data:
                    mov_y(0)
                else:
                    mov_y(1)
            elif 'z' in data:
                if 'forward' in data:
                    mov_z(0)
                else:
                    mov_z(1)
            elif 'stop' in data:
                mov_stop()
                
            elif 'path' in data:
                mov_path()
                
            elif 'shut' in data:
                break
        break
    finally:
        # Clean up the connection
        connection.close()
