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

X_SPEED = 20
Y_SPEED = 20
Z_SPEED = 10
def mov_x(du, di):
    if di == 1:
        di = di << 2
        
    msg = can.Message(arbitration_id=0x121, data=[1, 0, X_SPEED, 0, 0, 0, 0, di], extended_id=False)
    can0.send(msg)
    time.sleep(du)
    mov_stop()
    
def mov_y(du, di):
    if di == 1:
        di = di << 1
        
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, Y_SPEED, 0, 0, di], extended_id=False)
    can0.send(msg)
    time.sleep(du)
    mov_stop()
    
def mov_z(du, di):
    if di == 1:
        di = di 
        
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, 0, 0, Z_SPEED, di], extended_id=False)
    can0.send(msg)
    time.sleep(du)
    mov_stop()
    
def mov_stop():
    msg = can.Message(arbitration_id=0x121, data=[1, 0, 0, 0, 0, 0, 0, 0], extended_id=False)
    can0.send(msg)
    time.sleep(0.1)
    
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('169.254.52.228', 10000)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)
sock.listen(1)

while True:
##if True:
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
                    mov_x(5, 0)
                else:
                    mov_x(5, 1)
                        
            elif 'y' in data:
                if 'forward' in data:
                    mov_y(5, 0)
                else:
                    mov_y(5, 1)
            elif 'z' in data:
                if 'forward' in data:
                    mov_z(5, 0)
                else:
                    mov_z(5, 1)
            elif 'stop' in data:
                mov_stop()
            
    finally:
        # Clean up the connection
        connection.close()
