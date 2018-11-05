#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygame, sys


def main():
	pygame.init()
	screen = pygame.display.set_mode((600,800))
	font = pygame.font.SysFont(None, 24)
	clock = pygame.time.Clock()

	def _disptxt(s_arr):
		screen.fill((255,255,255))
		for i,s in enumerate(s_arr):
			screen.blit(font.render(s, True, (0,0,0)), (10, 10 + i*24))
		pygame.display.flip()

	pad_map = {
			pygame.K_UP: (-1,0,0),
			pygame.K_DOWN: (1,0,0),
			pygame.K_LEFT: (0,-1,0),
			pygame.K_RIGHT: (0,1,0),
			pygame.K_w: (0,0,1),
			pygame.K_s: (0,0,-1)
	}

	pygame.joystick.init()
	joystick_count = pygame.joystick.get_count()
	if joystick_count > 0:
		_disptxt(["Found %d joysticks." % joystick_count])
	else:
		_disptxt(["Found no joysticks.", "Exit..."])
		time.sleep(1)
		pygame.quit()
		return 1

	joystick = pygame.joystick.Joystick(0)
	joystick.init()
	axes = joystick.get_numaxes()
	buttons = joystick.get_numbuttons()
	hats = joystick.get_numhats()
	_disptxt([
		"Using joystick \"%s\"" % joystick.get_name(),
		"Number of axes: %d" % axes,
		"Number of buttons: %d" % buttons,
		"Number of hats: %d" % hats

	])


	running = True
	while running:
		b_msg = []
		ax_msg = []
		hat_msg = []
		for e in pygame.event.get():
			pressed = pygame.key.get_pressed()
			if e.type == pygame.QUIT or pressed[pygame.K_ESCAPE]:
				running = False

			# Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
			#if e.type == pygame.JOYBUTTONDOWN:
			#	b_msg.append("Joystick button pressed.")
			#if e.type == pygame.JOYBUTTONUP:
			#	b_msg.append("Joystick button released.")


		for i in range(axes):
			axis = joystick.get_axis(i)
			ax_msg.append("Axis %d value %.2f" % (i, axis))

		for i in range(buttons):
			button = joystick.get_button(i)
			b_msg.append("Button %d value %.2f" % (i, button))

		# Hat switch. All or nothing for direction, not like joysticks.
		# Value comes back in an array.
		for i in range(hats):
			hat = joystick.get_hat(i)
			hat_msg.append("Hat %d value %s" % (i, str(hat)))

		_disptxt(b_msg + hat_msg + ax_msg)
		clock.tick(60)

	pygame.quit()


if __name__ == '__main__':
	main()
