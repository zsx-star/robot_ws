import os
import can
import time

os.system('sudo ip link set can0 type can bitrate 500000')
os.system('sudo ifconfig can0 up')

can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan_ctypes')# socketcan_native

msg = can.Message(arbitration_id=0x121, data=[10, 12, 15, 19, 24, 30, 37, 1], extended_id=False)
can0.send(msg)

time.sleep(1)


msg = can.Message(arbitration_id=0x121, data=[1, 0, 50, 0, 0, 0, 0, 0], extended_id=False)
can0.send(msg)

time.sleep(2)

msg = can.Message(arbitration_id=0x121, data=[0, 0, 20, 0, 0, 0, 0, 1], extended_id=False)
can0.send(msg)

time.sleep(5)

msg = can.Message(arbitration_id=0x121, data=[10, 12, 15, 19, 24, 30, 37, 0], extended_id=False)
can0.send(msg)

time.sleep(2)
os.system('sudo ifconfig can0 down')
