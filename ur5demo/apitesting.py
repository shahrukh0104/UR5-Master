#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, urx, math, time, math3d

IP = "129.241.187.119"
#JOINTS_HOME = [5*math.pi/4, -3*math.pi/4, -3*math.pi/4, -math.pi/2, 3*math.pi/2, math.pi]
#JOINTS_HOME = [math.pi/4, -3*math.pi/4, -3*math.pi/4, -math.pi/2, 3*math.pi/2, math.pi]
JOINTS_HOME = [0, -3*math.pi/4, -3*math.pi/4, -math.pi/2, -math.pi/2, math.pi]




class RobotInitException(Exception):
    pass

class UR5TestController(object):

    def __init__(self, ip, joint_init=None):
        self.vel = 0.1
        self.acc = 1.0

        try:
            self.robot = urx.Robot(ip, useRTInterface=False)
            if joint_init:
                self.robot.movej(joint_init, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
            self.pose = self.robot.getl()
        except Exception, e:
            raise RobotInitException('Robot initiation failed: %s' % str(e))
    
    def test_csys(self, quadrant):
        theta_c = math.pi*quadrant/2
        pose = self.robot.getl()
        
        csys = math3d.Transform()
        csys.orient.rotate_zb(theta_c)
        self.robot.set_csys("csys", csys)
        self.robot.movel(pose, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)

        trans_base = self.robot.get_transform()


        trans = math3d.Transform(trans_base) #deep copy
        pos = pose[0:3]
        trans.pos = math3d.Vector(pos[0]+0.1, pos[1], pos[2])
        trans.orient.rotate_zb(math.pi/4)

        self.robot.movel(trans.pose_vector.tolist(), acc=0.1, vel=0.5, radius=0, wait=True, relative=False)


    def table_quadrant_test(self, quadrant):
        # quadrant = {0..3}
        theta_c = math.pi*quadrant/2
        r = 0.6

        joints0 = [theta_c, -3*math.pi/4, -3*math.pi/4, -math.pi/2, -math.pi/2, math.pi]
        self.robot.movej(joints0, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
        pose = self.robot.getl()
        pose[0] = r*math.cos(theta_c)
        pose[1] = r*math.sin(theta_c)
        pose[2] = 0
        self.robot.movel(pose, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
        print ('q %.1f: ' % quadrant) + str([('%.3f' % i) for i in self.robot.getl()])
        time.sleep(1)

    def table_end_poses(self):
        joints0 = [0, -3*math.pi/4, -3*math.pi/4, -math.pi/2, -math.pi/2, math.pi]
        self.robot.movej(joints0, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
        print 'pose: ' + str(self.robot.getl())
        time.sleep(1)
        joints1 = [33*math.pi/16, -3*math.pi/4, -3*math.pi/4, -math.pi/2, 27*math.pi/16, math.pi]
        self.robot.movej(joints1, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
        print 'pose: ' + str(self.robot.getl())

    def table_quadrant_test2(self, quadrant):
        # quadrant = {0..3}
        theta_c = math.pi*quadrant/2
        r = 0.6

        pose = self.robot.getl()
        pose[0] = r*math.cos(theta_c)
        pose[1] = r*math.sin(theta_c)
        self.robot.movel(pose, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
        print 'pose: ' + str(self.robot.getl())
        time.sleep(1)


    def test_circ_pose(self):
        theta_via = math.pi*2.75/2
        pose0 = [-0.424, -0.424, -0.097, -0.729, -1.761, 1.760]
        posevia = [0.6*math.cos(theta_via), 0.6*math.sin(theta_via), -0.097, 0, 0, 0]
        pose1 = [0.000, -0.600, -0.097, 0.000, 2.221, -2.221]
        #pose1 = [0.424, -0.424, -0.097, -0.729, 1.760, -1.759]

        self.robot.movel(pose0, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
        print 'Starting circular move...'
        self.robot.movec(posevia, pose1, acc=0.1, vel=0.1, radius=0.1, wait=True)
        time.sleep(1)
        self.robot.movel(pose1, acc=0.1, vel=0.1, radius=0, wait=True)
        

    def test_circ_pose2(self):
        pose0 = [-0.424, -0.424, -0.097, -0.729, -1.761, 1.760]
        posevia = [0.424, -0.424, -0.097, -0.729, 1.760, -1.759]
        #posevia = [0.000, -0.600, -0.097, 0.000, 2.221, -2.221]
        pose1 = [0.424, 0.424, -0.097, -1.482, 0.614, -0.613]
        #pose1 = [0.424, -0.424, -0.097, -0.729, 1.760, -1.759]

        self.robot.movel(pose0, acc=0.1, vel=0.5, radius=0, wait=True, relative=False)
        print 'Starting circular move...'
        self.robot.movec(posevia, pose1, acc=0.1, vel=0.1, radius=0, wait=True)
        time.sleep(1)
        self.robot.movel(pose1, acc=0.1, vel=0.1, radius=0, wait=True)

    def test_axes(self, axis):
        dist = 0.6
	pose = self.robot.getl()
	pose[0] = dist
        pose[1] = dist	

        if axis == 'x':
            pose[1] = 0
            #pose[2] = 0

            _theta = math.pi/2
            pose[3] = math.pi/2
            pose[4] = math.pi
            pose[5] = 0
#pose:[0.5921354182317952, 0.00589012470721734, -0.0973002219692428, -1.4823549412900907, 0.614936708121804, -0.6137840711781131]

        elif axis == 'y':
            pose[0] = 0	
        self.robot.movel(pose, acc=0.1, vel=0.1, wait=True)
        print 'pose: ' + str(self.robot.getl())
        time.sleep(2)
	

    def test_circ(self):
        pose0 = self.robot.getl()
        pose1 = [i for i in pose0]
        posevia = [i for i in pose0]

        r = 0.4
        t0 = 9*math.pi/8
        t1 = 13*math.pi/8
        tvia = t0 + (t1-t0) / 2

        pose0[0] = r*math.cos(t0)
        pose0[1] = r*math.sin(t0)
        self.robot.movel(pose0, acc=0.1, vel=0.1, wait=True)
        
        print 'pose: ' + str(pose0)
        print 'starting circular move'
        posevia[0] = r*math.cos(tvia)
        posevia[1] = r*math.sin(tvia)
        pose1[0] = r*math.cos(t1)
        pose1[1] = r*math.sin(t1)
        self.robot.movec(posevia, pose1, acc=0.1, vel=0.1, wait=True)
        print 'pose: ' + str(pose1)


    def test_speedl(self):

        n = 100
        vel = 0.08
        dtheta = 4*math.pi/n
        vels = [(-vel*math.sin(i*dtheta), vel*math.cos(i*dtheta), 0, 0, 0, 0)
                 for i in range(n)]

        dt = 0.3
        for v in vels:
            self.robot.speedl(v, 0.7, dt)
            time.sleep(dt)

    def test_movel(self):
        n = 10
        dt = 0.5

        pose = self.robot.getl()
        pose[0] -= 0.1*math.sin(0)
        pose[1] += 0.1*math.cos(1)
        self.robot.movel(pose, acc=0.1, vel=0.1, radius=0, wait=False)
        time.sleep(dt)
        for i in range(1, n):
            pose = self.robot.getl()
            pose[0] -= math.sin(math.pi*i/n)
            pose[1] += math.cos(math.pi*i/n)
            self.robot.movel(pose, acc=1, vel=0.1, radius=0.1, wait=False)
            time.sleep(dt)
        self.robot.stopl(0.2)


    def test_servoj(self):
        j0 = self.robot.getj()
        dtheta = math.pi/4
        j_incr = [dtheta]*6

        j = [i+math.pi/3 for i in j0]
        prog = "servoj([{},{},{},{},{},{}])".format(*j)
        self.robot.send_program(prog)        
        time.sleep(2)
        return

        j = j0
        for i in range(10):
            j = map(sum, zip(j, j_incr))
            prog = "servoj([{},{},{},{},{},{}])".format(*j)
            self.robot.send_program(prog)
            time.sleep(0.5)


    def velocity_control(self):
        if vels == None:
            n = 20
            vel = 0.05
            dtheta = math.pi/n
            vels = [(-vel*math.sin(i*dtheta), vel*math.cos(i*dtheta), 0, 0, 0, 0)
                     for i in range(n)]

    def test_circle(self):
        pass


    def follow_path(self, path):
        pass


    def move(self, vec):
        # check internal states
        # update pose etc
        self.pose = self.robot.getl()
        self.pose[0] += vec[0]
        self.pose[1] += vec[1]
        self.pose[2] += vec[2]

        if sum(map(abs, vec)) > 0:
            self.robot.movel(self.pose, acc=self.acc, vel=self.vel, radius=0.1, wait=False)
        else:
            self.robot.stopl(self.stop_acc)

    def get_tcp_force(self):
        return self.robot.get_tcp_force(wait=False)

    def get_2dvec_out(self):
        # Vector outwards from the robot in (x, y) coordinates
        theta_d = 0.02
        j1 = self.robot.getj()
        j2 = [theta+theta_d if i==1 else theta for i,theta in enumerate(j1)]
        p1 = self.robot.getl()

        # Make a small move outwards and inwards
        self.robot.movej(j2, acc=0.1, vel=0.5, radius=0, wait=True)
        p2 = self.robot.getl()
        self.robot.movej(j1, acc=0.1, vel=0.5, radius=0, wait=True)
        return (p1[0]-p2[0], p1[1]-p2[1])

    def cleanup(self):
        self.robot.cleanup()

if __name__ == '__main__':
    controller = None
    try:
        controller = UR5TestController(IP, joint_init=JOINTS_HOME)
        controller.test_csys(1)
        #controller.test_circ_pose()
        #controller.test_circ_pose2()
        #controller.table_end_poses()
        #controller.table_quadrant_test(0)
        #controller.table_quadrant_test(0)
        #controller.table_quadrant_test(0.5)
        #controller.table_quadrant_test(1)
        #controller.table_quadrant_test(1.5)
        #controller.table_quadrant_test(2)
        #controller.table_quadrant_test(2.5)
        #controller.table_quadrant_test(3)
        #controller.table_quadrant_test(3.5)
        #controller.test_circ()
        #controller.test_axes('x')
        #controller.test_axes('dummy')
        #controller.test_axes('y')
        #controller.test_speedl()
        #controller.test_servoj()
        #controller.test_movel()
    except Exception, e:
        print e

    if controller:
        controller.cleanup()
