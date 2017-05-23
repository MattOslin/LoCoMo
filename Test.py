import pygame
import Module
import time
import sys

from pygame.locals import *

pygame.init()

DISPLAYSURF = pygame.display.set_mode((300,300))
pygame.display.set_caption('LoCoMo Debugger')

mod = Module.Module("192.168.10.13")

mod.send_SetBrake(False);

while True:

	#get all the user events
	for event in pygame.event.get():
        #if the user wants to quit
		if event.type == QUIT:
			print("Stopping")
			mod.send_Stop()
			pygame.quit()
			sys.exit()
		elif event.type == KEYUP:
			mod.send_Stop()
		elif event.type == KEYDOWN:
			if event.key == K_LEFT:
				mod.send_PowTo(1000,5)
			elif event.key == K_RIGHT:
				mod.send_PowTo(-1000,5)
			elif event.key == K_SPACE:
				print("Checking position")
				mod.send_PosRequest(True)

	pygame.display.update()


