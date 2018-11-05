#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, config, math, time, traceback
from ur5controller import DemoController


def check_z_levels(controller):
    print 'Going to home pose.'
    controller.move_to_home_pose()
    time.sleep(1)

    r, theta, z = controller.current_cyl
    
    print "z-level 0"
    z = config.Z_LVL0
    move = controller.cylinder2cartesian(r, theta, z) + [0.2, 0.1, 0]
    controller.exec_move(move, wait=True)
    controller.set_cylinder_coords()
    dummy = raw_input('Press any key to continue...')

    print "z-level 1"
    z += config.BLOCK_DIM
    move = controller.cylinder2cartesian(r, theta, z) + [0.2, 0.1, 0]
    controller.exec_move(move, wait=True)
    controller.set_cylinder_coords()
    dummy = raw_input('Press any key to continue...')

    print "z-level 2"
    z += config.BLOCK_DIM
    move = controller.cylinder2cartesian(r, theta, z) + [0.2, 0.1, 0]
    controller.exec_move(move, wait=True)
    controller.set_cylinder_coords()
    dummy = raw_input('Press any key to continue...')

    print "z-level 3"
    z += config.BLOCK_DIM
    move = controller.cylinder2cartesian(r, theta, z) + [0.2, 0.1, 0]
    controller.exec_move(move, wait=True)
    controller.set_cylinder_coords()
    dummy = raw_input('Press any key to continue...')

    print "z-level 4"
    z += config.BLOCK_DIM
    move = controller.cylinder2cartesian(r, theta, z) + [0.2, 0.1, 0]
    controller.exec_move(move, wait=True)
    controller.set_cylinder_coords()
    dummy = raw_input('Press any key to continue...')


def test_unitary_moves(controller):
    print 'Going to home pose.'
    controller.move_to_home_pose()
    time.sleep(1)

    print 'Moving (+r, 0, 0)'
    controller.move((1,0,0), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (-r, 0, 0)'
    controller.move((-1,0,0), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (0, +theta, 0)'
    controller.move((0,1,0), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (0, -theta, 0)'
    controller.move((0,-1,0), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (0, 0, +z)'
    controller.move((0,0,1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (0, 0, -z)'
    controller.move((0,0,-1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()


def test_multi_moves(controller):
    print 'Going to home pose.'
    controller.move_to_home_pose()
    time.sleep(1)

    print 'Moving (+r, +theta, 0)'
    controller.move((1,1,0), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (-r, -theta, 0)'
    controller.move((-1,-1,0), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (+r, -theta, 0)'
    controller.move((1,-1,0), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (+r, 0, z)'
    controller.move((1,0,1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (-r, 0, z)'
    controller.move((-1,0,1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (0, theta, -z)'
    controller.move((0,1,-1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (0, -theta, -z)'
    controller.move((0,-1,-1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (r, theta, z)'
    controller.move((1,1,1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()

    print 'Moving (-r, theta, -z)'
    controller.move((-1,1,-1), t=2)
    time.sleep(1)
    controller.set_cylinder_coords()


def check_loop_constraints(controller):
    print 'Going to home pose.'
    controller.move_to_home_pose()
    time.sleep(1)

    controller.movec_hax(config.R_LVL0, config.THETA_EDGE_RIGHT, config.Z_LVL0, wait=True)
    controller.set_cylinder_coords()
    print 'Right edge, r-level 0, z-level 0. Cylinder coordinates: (%s)' % ','.join(['%.2f' % i for i in controller.current_cyl])     
    dummy = raw_input('Press any key to continue...')

    controller.movec_hax(config.R_LVL0, 0, config.Z_LVL0, wait=True)
    controller.set_cylinder_coords()
    print 'Middle, r-level 0, z-level 0. Cylinder coordinates: (%s)' % ','.join(['%.2f' % i for i in controller.current_cyl])     
    dummy = raw_input('Press any key to continue...')

    controller.movec_hax(config.R_LVL0, config.THETA_EDGE_LEFT, config.Z_LVL0, wait=True)
    controller.set_cylinder_coords()
    print 'Left edge, r-level 0, z-level 0. Cylinder coordinates: (%s)' % ','.join(['%.2f' % i for i in controller.current_cyl])     
    dummy = raw_input('Press any key to continue...')


def test_movec(controller):
    print 'Going to home pose.'
    controller.move_to_home_pose()
    time.sleep(1)

    controller.movec(config.R_LVL0, config.THETA_EDGE_RIGHT, config.Z_LVL0, wait=True)
    controller.set_cylinder_coords()
    print 'Right edge, r-level 0, z-level 0. Cylinder coordinates: (%s)' % ','.join(['%.2f' % i for i in controller.current_cyl])     
    dummy = raw_input('Press any key to continue...')

    controller.movec(config.R_LVL0, 0, config.Z_LVL0, wait=True)
    controller.set_cylinder_coords()
    print 'Middle, r-level 0, z-level 0. Cylinder coordinates: (%s)' % ','.join(['%.2f' % i for i in controller.current_cyl])     
    dummy = raw_input('Press any key to continue...')

    controller.movec(config.R_LVL0, config.THETA_EDGE_LEFT, config.Z_LVL0, wait=True)
    controller.set_cylinder_coords()
    print 'Left edge, r-level 0, z-level 0. Cylinder coordinates: (%s)' % ','.join(['%.2f' % i for i in controller.current_cyl])     
    dummy = raw_input('Press any key to continue...')



def four_stories(controller):
    print "Initial block placement."
    print "On each side"
    print "\t(r0, edge, z0)"
    print "\t(r1, edge, z0)"

    # buffer some params
    edge_left = config.THETA_EDGE_LEFT
    edge_right = config.THETA_EDGE_RIGHT
    r_startpick = 1.3
    r_endpick = 0.1
    z_endpick = 0.5

    vel = 0.2
    acc = 1.1*vel
    moves = []

    moves.append(controller.blocklevel2move(0.5, 0, 0, acc, vel, 0))
    moves += controller.pick_block(1, edge_left, 0, acc, vel)
    moves += controller.place_block(0, 0, 0, acc, vel)

    moves += controller.pick_block(1, edge_right, 0, acc, vel)
    moves.append(controller.blocklevel2move(1.1, edge_right/2, 2, acc, vel, 0.15))
    moves += controller.place_block(0, 0, 1, acc, vel)

    moves += controller.pick_block(0, edge_left, 0, acc, vel)
    moves.append(controller.blocklevel2move(0.1, edge_left/2, 3, acc, vel, 0.15))
    moves += controller.place_block(0, 0, 2, acc, vel)

    moves += controller.pick_block(0, edge_right, 0, acc, vel)
    moves.append(controller.blocklevel2move(0.1, edge_right/2, 4, acc, vel, 0.15))
    moves += controller.place_block(0, 0, 3, acc, vel)

    moves.append(controller.blocklevel2move(1.5, 0, 0.1, acc, vel, 0))

    dummy = raw_input("Press enter to start...")
    print 'Exectuting loop'
    controller.exec_moves(moves, wait=False)
    looping = True
    while looping:
        looping = controller.is_looping(1.0) # blocks for 1 sec

    moves.reverse()
    controller.exec_moves(moves, wait=False)
    looping = True
    while looping:
        looping = controller.is_looping(1.0) # blocks for 1 sec


def small_pyramid(controller):
    print "Initial block placement."
    print "On each side"
    print "\t(r0, edge, z0)"
    print "\t(r1, edge, z0)"
    print "\t(r0, edge, z1)"

    # buffer some params
    edge_left = config.THETA_EDGE_LEFT
    edge_right = config.THETA_EDGE_RIGHT
    r_startpick = 1.3
    r_endpick = 0.1
    z_endpick = 0.5

    vel = 0.2
    acc = 1.1*vel
    dtheta_lvl0 = config.BLOCK_DIM/config.R_LVL0 + math.pi/64 #medium gap
    moves = []

    moves.append(controller.blocklevel2move(0.5, 0, 0, acc, vel, 0)) #initial
    moves += controller.pick_block(0, edge_left, 1, acc, vel)
    moves.append(controller.blocklevel2move(0.1, edge_left/2, 2, acc, vel, 0.15))
    moves += controller.place_block(0, 0, 0, acc, vel)

    moves += controller.pick_block(0, edge_right, 1, acc, vel)
    moves.append(controller.blocklevel2move(0.1, (edge_right+dtheta_lvl0)/2, 2, acc, vel, 0.15))
    moves += controller.place_block(0, dtheta_lvl0, 0, acc, vel)

    moves.append(controller.blocklevel2move(1.1, (edge_left+dtheta_lvl0)/2, 0.5, acc, vel, 0.15))
    moves += controller.pick_block(1, edge_left, 0, acc, vel)
    moves += controller.place_block(0, -dtheta_lvl0, 0, acc, vel)

    moves.append(controller.blocklevel2move(1.1, (edge_right-dtheta_lvl0)/2, 0.5, acc, vel, 0.15))
    moves += controller.pick_block(1, edge_right, 0, acc, vel)
    moves.append(controller.blocklevel2move(1.1, edge_right/2, 2, acc, vel, 0.15))
    moves += controller.place_block(0, 0.5*dtheta_lvl0, 1, acc, vel)

    moves += controller.pick_block(0, edge_left, 0, acc, vel)
    moves.append(controller.blocklevel2move(0.1, edge_left/2, 2, acc, vel, 0.15))
    moves += controller.place_block(0, -0.5*dtheta_lvl0, 1, acc, vel)

    moves.append(controller.blocklevel2move(1.1, (edge_right-dtheta_lvl0)/2, 0.5, acc, vel, 0.15))
    moves += controller.pick_block(0, edge_right, 0, acc, vel)
    moves.append(controller.blocklevel2move(0.1, edge_right/2, 3, acc, vel, 0.15))
    moves += controller.place_block(0, 0, 2, acc, vel)

    moves.append(controller.blocklevel2move(1.5, 0, 0.1, acc, vel, 0))

    dummy = raw_input("Press enter to start...")
    print 'Exectuting loop'
    controller.exec_moves(moves, wait=False)
    looping = True
    while looping:
        looping = controller.is_looping(1.0) # blocks for 1 sec

    moves.reverse()
    controller.exec_moves(moves, wait=False)
    looping = True
    while looping:
        looping = controller.is_looping(1.0) # blocks for 1 sec



def unit_tests(func_list, controller):
    running = True
    while running:
        for i,tup in enumerate(func_list):
            print '(%d) %s' % (i, tup[0])
        c = raw_input('Select test or type q to quit... ')
        if c == 'q':
            running = False
        else:
            try:
                idx = int(c)
                f = func_list[idx][1]
                f(controller)
            except Exception, e:
                print e
                print traceback.format_exc()
                running = False


if __name__ == '__main__':
    controller = None
    print "Initializing..."
    try:
        controller = DemoController(config)
        controller.set_freedrive(False)
        time.sleep(1)
        controller.calibrate_cylinder_sys()
    except Exception, e:
        print e
        print traceback.format_exc()

    tests = [
        ("Check z-levels", check_z_levels),
        ("Test unitary moves.", test_unitary_moves),
        ("Test multi-directional moves", test_multi_moves),
        ("Check loop constraints", check_loop_constraints),
        ("Test URScript's movec", test_movec),
        ("Build four stories high", four_stories),
        ("Build small pyramid", small_pyramid)
    ]
    unit_tests(tests, controller)

    time.sleep(1)
    if controller:
        controller.cleanup()
    sys.exit(0)
