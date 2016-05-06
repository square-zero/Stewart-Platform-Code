# Ryan Cysewski
# EE 449
# To install pygame, copy and paste the following into your terminal:
# "sudo apt-get install python-pygame"

import pygame
pygame.init()

obstacle = True

if obstacle is True:
	# Type the name of the audio file you wish to play below
	pygame.mixer.music.load("Mamma Mia.mp3")
	pygame.mixer.music.play()
	pygame.event.wait()
