#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pygame, sys, math, time, config
from ur5controller import DemoController


class UR5Demo(object):

    def __init__(self):
        # Set up pygame, GUI and controls
        pygame.init()
        self.screen = pygame.display.set_mode(config.SCREEN_DIM)
        pygame.display.set_caption("UR5 Demo")
        self.font = pygame.font.SysFont(None, config.FONTSIZE)

        pygame.joystick.init()
        self.padcount = pygame.joystick.get_count()
        self.gamepad = None
        self.hatcode = config.HATCODE
        self.hat_map = config.HAT_MAP
        self.but_map = config.BUTTON_MAP

        # move vectors are in cylinder coords
        self.kb_map = {
                pygame.K_UP: (-1,0,0),
                pygame.K_DOWN: (1,0,0),
                pygame.K_LEFT: (0,-1,0),
                pygame.K_RIGHT: (0,1,0),
                pygame.K_w: (0,0,1),
                pygame.K_s: (0,0,-1)
        }

        self.clock = pygame.time.Clock()
        self.sig_quit = False
        self.loop_speed = config.DEFAULT_LOOP_VEL
        self.controller = None

    def _disptxt(self, s_arr):
        self.screen.fill(config.BG_COL)
        for i,s in enumerate(s_arr):
            self.screen.blit(self.font.render(s, True, config.FONT_COL), (10, 10 + i*config.FONTSIZE))
        pygame.display.flip()

    def quit(self, error=None):
        msg = []
        if error:
            msg.append(error)

        msg.append("Stopping robot...")
        self._disptxt(msg)
        try:
            self.controller.robot.stopl(acc=0.2)
        except Exception, e:
            msg.append("Could not stop robot: %s" % e)
        self._disptxt(msg)

        msg.append("Setting robot to freedrive mode...")
        self._disptxt(msg)
        try:
            self.controller.set_freedrive(True)
        except Exception, e:
            msg.append("Could not set freedrive mode: %s" % e)
        self._disptxt(msg)
        time.sleep(1) # give freedrive mode some time to set
        
        msg.append("Disconnecting robot...")
        self._disptxt(msg)
        if self.controller:
            self.controller.cleanup()
        time.sleep(1) # Give the UR5 threads a chance to terminate
        msg.append("Exit...")
        self._disptxt(msg)
        pygame.quit()
        if error:
            return 1
        else:
            return 0


    def go_home_pos(self):
        msg = ["Going to home configuration..."]
        self._disptxt(msg)
        pose = self.controller.cylinder2cartesian(config.R_HOME, config.THETA_HOME, config.Z_HOME)
        move = pose + [0.5, 0.1, 0]
        try:
            self.controller.exec_move(move, wait=True)
        except Exception, e:
            msg.append("Error: %s" % e)
            self._disptxt(msg)
            time.sleep(2)


    def freedrive(self):
        self.controller.set_freedrive(True)
        self._disptxt(["Freedrive mode"])
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True
            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            self.clock.tick(config.MENU_FPS)
        self.controller.set_freedrive(False)

    def adjust_loop_speed(self):
        incr = 0.05
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    self.sig_quit = True
            if pressed[pygame.K_PLUS] or pressed[pygame.K_KP_PLUS]:
                self.loop_speed += incr
                self.loop_speed = min(self.loop_speed, config.LOOP_VEL_MAX)
            elif pressed[pygame.K_MINUS] or pressed[pygame.K_KP_MINUS]:
                self.loop_speed -= incr
                self.loop_speed = max(self.loop_speed, config.LOOP_VEL_MIN)
            elif pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            self._disptxt([
                "Current speed: %.2f m/s" % self.loop_speed,
                "(+) Increase speed",
                "(-) Decrease speed"
            ])
            self.clock.tick(config.MENU_FPS)

    def run_loop1(self):
        vel = self.loop_speed
        acc = 1.1*vel
        edge_left = config.THETA_EDGE_LEFT
        edge_right = config.THETA_EDGE_RIGHT
        
        moves = []
        moves.append(self.controller.blocklevel2move(0.5, 0, 0, acc, vel, 0))
        moves += self.controller.pick_block(1, edge_left, 0, acc, vel)
        moves += self.controller.place_block(0, 0, 0, acc, vel)
        moves += self.controller.pick_block(1, edge_right, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, edge_right/2, 2, acc, vel, 0.15))
        moves += self.controller.place_block(0, 0, 1, acc, vel)
        moves += self.controller.pick_block(0, edge_left, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, edge_left/2, 3, acc, vel, 0.15))
        moves += self.controller.place_block(0, 0, 2, acc, vel)
        moves += self.controller.pick_block(0, edge_right, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, edge_right/2, 4, acc, vel, 0.15))
        moves += self.controller.place_block(0, 0, 3, acc, vel)
        moves.append(self.controller.blocklevel2move(1.5, 0, 0.1, acc, vel, 0))

        loopcount = 0.0
        self._disptxt(["Running loop \"four blocks high\"", "Loopcount: %d" % math.floor(loopcount)])
        self.controller.exec_moves(moves, wait=False)
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True
            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            if not self.controller.is_looping(1.0):
                moves.reverse()
                self.controller.exec_moves(moves, wait=False) # try-catch?
                loopcount += 0.5
                self._disptxt(["Running loop \"four blocks high\"", "Loopcount: %d" % math.floor(loopcount)])

            #self.clock.tick(config.MENU_FPS) # we dont need this here, check loop blocks for 1 sec
        self.controller.robot.stopj(acc)

    def run_loop2(self):
        vel = self.loop_speed
        acc = 1.1*vel
        dtheta_lvl0 = config.BLOCK_DIM/config.R_LVL0 + math.pi/64 #medium gap
        edge_left = config.THETA_EDGE_LEFT
        edge_right = config.THETA_EDGE_RIGHT

        moves = []
        moves.append(self.controller.blocklevel2move(0.5, 0, 0, acc, vel, 0)) #initial
        moves += self.controller.pick_block(0, edge_left, 1, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, edge_left/2, 2, acc, vel, 0.15))
        moves += self.controller.place_block(0, 0, 0, acc, vel)
        moves += self.controller.pick_block(0, edge_right, 1, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, (edge_right+dtheta_lvl0)/2, 2, acc, vel, 0.15))
        moves += self.controller.place_block(0, dtheta_lvl0, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_left+dtheta_lvl0)/2, 0.5, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_left, 0, acc, vel)
        moves += self.controller.place_block(0, -dtheta_lvl0, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta_lvl0)/2, 0.5, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_right, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, edge_right/2, 2, acc, vel, 0.15))
        moves += self.controller.place_block(0, 0.5*dtheta_lvl0, 1, acc, vel)
        moves += self.controller.pick_block(0, edge_left, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, edge_left/2, 2, acc, vel, 0.15))
        moves += self.controller.place_block(0, -0.5*dtheta_lvl0, 1, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta_lvl0)/2, 0.5, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_right, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, edge_right/2, 3, acc, vel, 0.15))
        moves += self.controller.place_block(0, 0, 2, acc, vel)
        moves.append(self.controller.blocklevel2move(1.5, 0, 0.1, acc, vel, 0))


        loopcount = 0.0
        self._disptxt(["Running loop \"small pyramid\"", "Loopcount: %d" % math.floor(loopcount)])
        self.controller.exec_moves(moves, wait=False)
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True
            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            if not self.controller.is_looping(1.0):
                moves.reverse()
                self.controller.exec_moves(moves, wait=False)
                loopcount += 0.5
                self._disptxt(["Running loop \"small pyramid\"", "Loopcount: %d" % math.floor(loopcount)])
        self.controller.robot.stopj(acc)


    def run_loop3(self):
        vel = self.loop_speed
        acc = 1.1*vel
        dtheta = 0.5*config.BLOCK_DIM/config.R_LVL0 + math.pi/128 #small gap
        edge_left = config.THETA_EDGE_LEFT
        edge_right = config.THETA_EDGE_RIGHT
        
        moves = []
        #level 1
        moves.append(self.controller.blocklevel2move(1.5, 0, 0, acc, vel, 0))
        moves += self.controller.pick_block(1, edge_left, 2, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_left-dtheta)/2, 2.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, -dtheta, 0, acc, vel)

        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta)/2, 2.0, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_right, 2, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_right+dtheta)/2, 2.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, dtheta, 0, acc, vel)

        #level 2
        moves.append(self.controller.blocklevel2move(1.1, (edge_left+dtheta)/2, 2.0, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_left, 2, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, (edge_left-dtheta)/2, 2.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, -dtheta, 1, acc, vel)

        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta)/2, 2.0, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_right, 2, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, (edge_right+dtheta)/2, 2.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, dtheta, 1, acc, vel)

        #level 3
        moves.append(self.controller.blocklevel2move(1.1, (edge_left+dtheta)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_left, 1, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_left-dtheta)/2, 2.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, -dtheta, 2, acc, vel)

        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_right, 1, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_right+dtheta)/2, 2.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, dtheta, 2, acc, vel)

        #level 4
        moves.append(self.controller.blocklevel2move(1.1, (edge_left+dtheta)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_left, 1, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, (edge_left-dtheta)/2, 3.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, -dtheta, 3, acc, vel)

        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_right, 1, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, (edge_right+dtheta)/2, 3.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, dtheta, 3, acc, vel)

        #level 5
        moves.append(self.controller.blocklevel2move(1.1, (edge_left+dtheta)/2, 1.0, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_left, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_left-dtheta)/2, 4.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, -dtheta, 4, acc, vel)

        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta)/2, 1.0, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_right, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(1.1, (edge_right+dtheta)/2, 4.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, dtheta, 4, acc, vel)

        #level 6
        moves.append(self.controller.blocklevel2move(1.1, (edge_left+dtheta)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_left, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, (edge_left-dtheta)/2, 5.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, -dtheta, 5, acc, vel)

        moves.append(self.controller.blocklevel2move(1.1, (edge_right-dtheta)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_right, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(0.1, (edge_right+dtheta)/2, 5.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, dtheta, 5, acc, vel)

        #end
        moves.append(self.controller.blocklevel2move(1.1, 2*dtheta, 0.5, acc, vel, 0))

        loopcount = 0.0
        self._disptxt(["Running loop \"tower\"", "Loopcount: %d" % math.floor(loopcount)])
        self.controller.exec_moves(moves, wait=False)
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True
            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            if not self.controller.is_looping(1.0):
                moves.reverse()
                self.controller.exec_moves(moves, wait=False) # try-catch?
                loopcount += 0.5
                self._disptxt(["Running loop \"tower\"", "Loopcount: %d" % math.floor(loopcount)])

            #self.clock.tick(config.MENU_FPS) # we dont need this here, check loop blocks for 1 sec
        self.controller.robot.stopj(acc)


    def run_loop4(self):
        vel = self.loop_speed
        acc = 1.1*vel
        nblocks = 12
        edge_left = config.THETA_EDGE_LEFT
        edge_right = config.THETA_EDGE_RIGHT
        theta0 = config.THETA_EDGE_LEFT
        dtheta = (edge_right-edge_left) / (nblocks-1)
        
        moves = []
        moves.append(self.controller.blocklevel2move(1.5, 0, 0, acc, vel, 0))
        moves += self.controller.pick_block(1, edge_left, 1, acc, vel)
        theta_next = theta0 + 5*dtheta
        moves.append(self.controller.blocklevel2move(1.1, (edge_left+theta_next)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)
        moves += self.controller.pick_block(0, theta_next, 1, acc, vel)
        theta_next += dtheta
        moves.append(self.controller.blocklevel2move(0, theta_next, 1.8, acc, vel, 0.03))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)

        moves.append(self.controller.blocklevel2move(1.8, (edge_right+theta_next)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_right, 1, acc, vel)
        theta_next = theta0 + 7*dtheta
        moves.append(self.controller.blocklevel2move(1.1, (edge_right+theta_next)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)
        moves += self.controller.pick_block(0, theta_next, 1, acc, vel)
        theta_next -= 3*dtheta
        moves.append(self.controller.blocklevel2move(0, theta_next, 1.8, acc, vel, 0.03))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)

        moves.append(self.controller.blocklevel2move(1.1, (edge_left+theta_next)/2, 1.5, acc, vel, 0.15))
        moves += self.controller.pick_block(0, edge_left, 1, acc, vel)
        theta_next = theta0 + 3*dtheta
        moves.append(self.controller.blocklevel2move(0.1, (edge_left+theta_next)/2, 1.2, acc, vel, 0.1))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)
        moves += self.controller.pick_block(0, theta_next, 1, acc, vel)
        theta_next += 5*dtheta
        moves.append(self.controller.blocklevel2move(0, theta_next, 1.8, acc, vel, 0.03))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)

        moves.append(self.controller.blocklevel2move(1.2, (edge_right+theta_next)/2, 1.5, acc, vel, 0.1))
        moves += self.controller.pick_block(0, edge_right, 1, acc, vel)
        theta_next = theta0 + 9*dtheta
        moves.append(self.controller.blocklevel2move(0.1, (edge_right+theta_next)/2, 1.5, acc, vel, 0.1))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)
        moves += self.controller.pick_block(0, theta_next, 1, acc, vel)
        theta_next -= 7*dtheta
        moves.append(self.controller.blocklevel2move(0, theta_next, 1.8, acc, vel, 0.03))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)

        moves.append(self.controller.blocklevel2move(2.0, (edge_left+theta_next)/2, 0.0, acc, vel, 0.15))
        moves += self.controller.pick_block(1, edge_left, 0, acc, vel)
        theta_next = theta0 + dtheta
        moves.append(self.controller.blocklevel2move(1.2, theta_next, 0.5, acc, vel, 0.03))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)
        
        moves.append(self.controller.blocklevel2move(1.1, (edge_right+theta_next)/2, 0.0, acc, vel, 0.15))
        theta_next += 9*dtheta
        moves += self.controller.pick_block(1, edge_right, 0, acc, vel)
        moves.append(self.controller.blocklevel2move(1.2, theta_next, 0.5, acc, vel, 0.03))
        moves += self.controller.place_block(0, theta_next, 0, acc, vel)

        moves.append(self.controller.blocklevel2move(1.5, 0, 0, acc, vel, 0))


        loopcount = 0.0
        self._disptxt(["Running loop \"spread out\"", "Loopcount: %d" % math.floor(loopcount)])
        self.controller.exec_moves(moves, wait=False)
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True
            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            if not self.controller.is_looping(1.0):
                moves.reverse()
                self.controller.exec_moves(moves, wait=False) # try-catch?
                loopcount += 0.5
                self._disptxt(["Running loop \"spread out\"", "Loopcount: %d" % math.floor(loopcount)])

            #self.clock.tick(config.MENU_FPS) # we dont need this here, check loop blocks for 1 sec
        self.controller.robot.stopj(acc)


    def loop_init(self, block_init_msg, loopnum):
        msg = [
            "Current loop speed is %.2f m/s" % self.loop_speed,
            "Start loop? [ENTER]"
        ]
        self._disptxt(block_init_msg + msg)
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True

            if pressed[pygame.K_RETURN]:
                if loopnum == 1:
                    self.run_loop1()
                elif loopnum == 2:
                    self.run_loop2()
                elif loopnum == 3:
                    self.run_loop3()
                elif loopnum == 4:
                    self.run_loop4()
                running = False

            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            self.clock.tick(config.MENU_FPS)

    def gamepad_control(self):
        t = time.time()
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True
            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            vec = (0,0,0)
            if self.hatcode >= 0:
                hx, hy = self.gamepad.get_hat(self.hatcode)
                mx = [hx*i for i in self.hat_map[config.HX]]
                my = [hy*i for i in self.hat_map[config.HY]]
                vec = map(sum, zip(vec, mx, my))
            for m in (self.but_map[code] for code in self.but_map if self.gamepad.get_button(code) > 0):
                vec = map(sum, zip(vec, m))

            new_t = time.time()
            dt = max(new_t-t, 1/config.CTR_FPS)
            t = new_t

            try:
                self.controller.update(tuple(vec), dt)
            except Exception, e:
                self._disptxt(["Error: %s" % e])
                time.sleep(2)
                running = False

            f = self.controller.zforce
            self._disptxt([
                "Gamepad control \"%s\"" % self.gamepad.get_name(),
                " ",
                "move vec: %s" % str(vec),
                "dt: %.3f" % dt,
                "force z: %s %.1f" % (" " if f>0 else "", f)
            ])
            self.clock.tick(config.CTR_FPS)

    def keyboard_control(self):
        t = time.time()
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                   self.sig_quit = True
            if pressed[pygame.K_ESCAPE] or self.sig_quit:
                running = False

            vec = (0,0,0)
            for m in (self.kb_map[key] for key in self.kb_map if pressed[key]):
                vec = map(sum, zip(vec, m))

            new_t = time.time()
            dt = max(new_t-t, 1/config.CTR_FPS)
            t = new_t

            try:
                self.controller.update(tuple(vec), dt)
            except Exception, e:
                self._disptxt(["Error: %s" % e])
                time.sleep(2)
                running = False

            f = self.controller.zforce
            self._disptxt([
                "Keyboard control",
                "Use arrow keys, [w], and [s]",
                " ",
                "move vec: %s" % str(vec),
                "dt: %.3f" % dt,
                "force z: %s %.1f" % (" " if f>0 else "", f)
            ])
            self.clock.tick(config.CTR_FPS)


    def main_menu(self):
        menu = [
            "Go to [h]ome position",
            "[f]reedrive mode",
            "Adjust loop [s]peed",
            "Start loop [1]: four blocks high",
            "Start loop [2]: small pyramid",
            "Start loop [3]: tower",
            "Start loop [4]: spread out",
            "Manual [c]ontrol",
            "[q]uit"
        ]
        self._disptxt(menu)
        running = True
        while running:
            pressed = pygame.key.get_pressed()
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    self.sig_quit = True

            if pressed[pygame.K_h]:
                self.go_home_pos()
                self._disptxt(menu)

            elif pressed[pygame.K_f]:
                self.freedrive()
                self._disptxt(menu)

            elif pressed[pygame.K_s]:
                self.adjust_loop_speed()
                self._disptxt(menu)

            elif pressed[pygame.K_1]:
                _instr = [
                    "Initital block configuration for loop \"four blocks high\":",
                    "On each side",
                    "    (r0, edge, z0)",
                    "    (r1, edge, z0)",
                    " "
                ]
                self.loop_init(_instr, 1)
                self._disptxt(menu)

            elif pressed[pygame.K_2]:
                _instr = [
                    "Initital block configuration for loop \"small pyramid\":",
                    "On each side",
                    "    (r0, edge, z0)",
                    "    (r1, edge, z0)",
                    "    (r0, edge, z1)",
                    " "
                ]
                self.loop_init(_instr, 2)
                self._disptxt(menu)

            elif pressed[pygame.K_3]:
                _instr = [
                    "Initital block configuration for loop \"tower\":",
                    "On each side",
                    "    (r0, edge, z0)",
                    "    (r1, edge, z0)",
                    "    (r0, edge, z1)",
                    "    (r1, edge, z1)",
                    "    (r0, edge, z2)",
                    "    (r1, edge, z2)",
                    " "
                ]
                self.loop_init(_instr, 3)
                self._disptxt(menu)

            elif pressed[pygame.K_4]:
                _instr = [
                    "Initital block configuration for loop \"spread out\":",
                    "On each side",
                    "    (r0, edge, z0)",
                    "    (r1, edge, z0)",
                    "    (r0, edge, z1)",
                    "    (r1, edge, z1)",
                    "    (r0, edge, z2)",
                    "    (r1, edge, z2)",
                    " "
                ]
                self.loop_init(_instr, 4)
                self._disptxt(menu)

            elif pressed[pygame.K_c]:
                if self.gamepad:
                    self.gamepad_control()
                else:
                    self.keyboard_control()
                self._disptxt(menu)

            elif pressed[pygame.K_q] or self.sig_quit:
                running = False

            self.clock.tick(config.MENU_FPS)

        return self.quit()
        # break loop on quit, or return quit() on error

    def run(self):
        msg = []
        msg.append("Connecting robot...")
        self._disptxt(msg)
        try:
            self.controller = DemoController(config)
            msg.append("Freedrive mode off...")
            self._disptxt(msg)
            self.controller.set_freedrive(False)
            time.sleep(0.5)
            msg.append("Going to home position...")
            msg.append("Calibrating cylinder coordinate system...")
            self._disptxt(msg)
            self.controller.calibrate_cylinder_sys()
        except Exception, e:
            return self.quit(error=("Error: %s" % e))
        msg.append("Calibration done.")
        self._disptxt(msg)

        if self.padcount > 0:
            msg.append("Found %d gamepad" % self.padcount)
            msg.append("Initializing gamepad..")
            self._disptxt(msg)
            try:
                self.gamepad = pygame.joystick.Joystick(0)
                self.gamepad.init()
            except Exception, e:
                return self.quit(error=("Error: %s" % e))
            msg.append("Gamepad \"%s\" initialized" % self.gamepad.get_name())
        else:
            msg.append("Found no gamepads")
            msg.append("Using keyboard for manual control")
        
        self._disptxt(msg)
        time.sleep(1)
        msg.append("Starting application...")
        self._disptxt(msg)
        time.sleep(1)

        return self.main_menu()


if __name__ == '__main__':
    demo = UR5Demo()
    sys.exit(demo.run())
