import socket
import time
import struct

class Module(object):
	def __init__(self, ip):
		self.ip = ip
		self.port = 2390
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.settimeout(2) # in seconds

	def send_BatteryRequest(self):
		msg = struct.pack("<B", 101)
		self.sock.sendto(msg, (self.ip, self.port))
		try:
			data = self.sock.recvfrom(256)
			(msg_id, batt_level) = struct.unpack("<Bf",data[0])
			print("Battery Voltage: {:4.2f}".format(batt_level))
		except socket.timeout:
			print("Not responding")

	def send_PosRequest(self):
		msg = struct.pack("<B", 122)
		self.sock.sendto(msg, (self.ip, self.port))
		try:
			data = self.sock.recvfrom(256)
			(msg_id, pos) = struct.unpack("<Bi",data[0])
			print("Position: {}".format(pos))
		except socket.timeout:
			print("Not responding")

	def send_PowTo(self, velo):
		msg = struct.pack("<Bf", 112, velo)
		self.sock.sendto(msg, (self.ip, self.port))

	def send_Stop(self):
		msg = struct.pack("<B", 111)
		self.sock.sendto(msg, (self.ip, self.port))

if __name__ == '__main__':
	mod = Module("192.168.10.12")
	
	mod.send_Stop

	print("Check the battery condition")
	mod.send_BatteryRequest()

	print("Checking encoder")
	mod.send_PosRequest()
