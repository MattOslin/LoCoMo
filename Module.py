import socket
import time
import struct
import matplotlib.pyplot as plt

class Module(object):
	def __init__(self, ip):
		self.ip = ip
		self.port = 2390
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.settimeout(1) # in seconds

	def send_BatteryRequest(self):
		msg = struct.pack("<B", 101)
		self.sock.sendto(msg, (self.ip, self.port))
		try:
			data = self.sock.recvfrom(256)
			(msg_id, batt_level) = struct.unpack("<Bf",data[0])
			print("Battery Voltage: {:4.2f}".format(batt_level))
		except socket.timeout:
			print("Not responding")

	def send_PosRequest(self, printing=False):
		msg = struct.pack("<B", 122)
		self.sock.sendto(msg, (self.ip, self.port))
		try:
			data = self.sock.recvfrom(256)
			(msg_id, pos) = struct.unpack("<Bf",data[0])
			if printing:
				print("Position: {:4.2f}".format(pos))
			return pos
		except socket.timeout:
			print("Not responding")
			return None

	def send_VelRequest(self):
		msg = struct.pack("<B", 90)
		self.sock.sendto(msg, (self.ip, self.port))
		try:
			data = self.sock.recvfrom(256)
			(msg_id, vel) = struct.unpack("<Bf",data[0])
			print("Velocity: {:4.2f}".format(vel))
		except socket.timeout:
			print("Not responding")

	def send_PowTo(self, velo, timeout):
		msg = struct.pack("<Bif", 112, velo, timeout)
		self.sock.sendto(msg, (self.ip, self.port))

	def send_SetBrake(self, brake):
		msg = struct.pack("<B?", 52, brake)
		self.sock.sendto(msg, (self.ip, self.port))

	def send_PosTo(self, pos, timeout):
		msg = struct.pack("<Bff", 84, pos, timeout)
		self.sock.sendto(msg, (self.ip, self.port))

	def send_Traj(self, a, b, c, d, e, f, dur):
		msg = struct.pack("<Bfffffff", 107, a, b, c, d, e, f, dur)
		self.sock.sendto(msg, (self.ip, self.port))

	def send_Stop(self):
		msg = struct.pack("<B", 111)
		self.sock.sendto(msg, (self.ip, self.port))

if __name__ == '__main__':
	mod = Module("192.168.10.12")
	#mod = Module("192.168.10.18")

	mod.send_Stop()

	mod.send_SetBrake(True)

	print("Checking encoder")
	mod.send_PosRequest(True)

	print("Zeroing")
#	mod.send_PosTo(0,5)

	#mod.send_Traj(0,0,0,0,0,0,5)
	mod.send_PowTo(800,5)
	time.sleep(5)
	
#	print("Oscillating")
#	mod.send_Traj(0.0125,-0.0982,0.0584,0.6936,0.0300,-1.5068,3.1416)
#	time.sleep(3.1416)
#	mod.send_Traj(-0.0125,0.0982,-0.0584,-0.6936,-0.0300,1.5068,3.1416)
#	time.sleep(3.1416)
#	mod.send_Traj(0.0125,-0.0982,0.0584,0.6936,0.0300,-1.5068,3.1416)
#	time.sleep(3.1416)
#	mod.send_Traj(-0.0125,0.0982,-0.0584,-0.6936,-0.0300,1.5068,3.1416)
#	time.sleep(3.1416)
#	mod.send_Traj(0.0125,-0.0982,0.0584,0.6936,0.0300,-1.5068,3.1416)
#	time.sleep(3.1416)
#	mod.send_Traj(-0.0125,0.0982,-0.0584,-0.6936,-0.0300,1.5068,3.1416)
#	time.sleep(3.1416)

#	positions = []
#	T = []
#	start = time.time()
#	duration = 5 # seconds
#	hz = 20.0
#	while time.time()-start < duration:
#		positions.append(mod.send_PosRequest())
#		T.append(time.time()-start)
#		time.sleep(1/hz)

	print("Stopping")
	mod.send_Stop()
#	plt.figure()
#	plt.plot(T, positions)
#	plt.show()

	
