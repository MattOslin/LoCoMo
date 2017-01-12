import Module
import time

mod = Module.Module("192.168.10.14")
print("Check the battery condition")
mod.send_BatteryRequest()
time.sleep(1)	

print("Setting Power to 500")
mod.send_PowTo(500)
time.sleep(4)

print("Setting Power to -500")
mod.send_PowTo(-500)
time.sleep(4)

print("Stopping")
mod.send_Stop()
