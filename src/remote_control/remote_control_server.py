#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import socket


class ReoteControl:
	def __init__(self,host,port):
		self.addr = (host,port)
		self.tcpSerSock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		self.tcpSerSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
		self.tcpSerSock.bind(self.addr)
		self.cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	
		
	def run(self):
		self.tcpSerSock.listen(5)
		while not rospy.is_shutdown():
			tcpCliSock,client_addr = self.tcpSerSock.accept() 
			print('connect ok addr:',client_addr)
			while not rospy.is_shutdown():
				data = tcpCliSock.recv(8)
				if not data:
					break
				print(data.decode(),type(data))
				self.parse(data)
			tcpCliSock.close()
		self.tcpSerSock.close()
		
	def parse(self,data):
		index = data.find('<robot>',0,len(data))
		if(index == -1):
			return
		val = int(data[index+7])
		
		cmd = Twist()
		if(val == 1):
			cmd.linear.x = 0.5
		elif(val == 2):
			cmd.linear.x = -0.5
		elif(val ==3):
			cmd.angular.z = 0.6
		elif(val ==4):
			cmd.angular.z = -0.6
		self.cmd_publisher.publish(cmd)

def main():
	rospy.init_node('remote_control_node', anonymous=True)
	
	remote_control = ReoteControl('',8008)
	remote_control.run()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
