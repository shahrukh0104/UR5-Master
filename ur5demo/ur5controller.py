import urx, math, math3d, time


class UR5Exception(Exception):
	pass


class DemoController(object):
	"""Implements control of the UR5 in cylinder coordinates (corresponding to attached table).
	   
	-Checks bounds of the arc section
	-Methods for placing blocks
	"""

	def __init__(self, config):
		"""Initializes the controller with parameters specified in config"""
		self.debug = config.DEBUG
		try:
			self.robot = urx.Robot(config.UR5_IP, useRTInterface=config.USE_FORCE_MONITOR) #TODO or hostname
		except Exception, e:
			raise UR5Exception('Robot initiation failed: %s' % str(e))

		# Cylinder coordinate system params
		self.quadrant = config.TABLE_QUADRANT
		self.phi = self.quadrant*math.pi/2
		self.cyl_offset = config.TABLE_ORIGO_OFFSET
		self.cyl_ox = self.cyl_offset[0]*math.cos(self.phi)
		self.cyl_oy = self.cyl_offset[1]*math.sin(self.phi)

		# Home pose params
		self.j_home = config.JOINTS_HOME
		self.r_home = config.R_HOME
		self.theta_home = config.THETA_HOME
		self.z_home = config.Z_HOME

		# Force monitor params
		self.force_mon = config.USE_FORCE_MONITOR
		self.force_constraint = config.FORCE_CONSTRAINT
		self.t_force = config.T_FORCE_VIOLATION

		# Manual control velocity params
		self.vel = config.VEL
		self.vel_z = config.VEL_Z
		self.acc = config.ACC
		self.stop_acc = config.STOP_ACC
		self.t_chcmd = config.T_DIR_CHANGE_COOLDOWN
		self.prev_vec = (0,0,0)
		self.zforce = 0.0
		self.t_ch = 0

		# Manual control constraints
		self.r_min = config.R_MIN
		self.r_max = config.R_MAX
		self.theta_min = config.THETA_MIN
		self.theta_max = config.THETA_MAX
		self.z_min = config.Z_MIN
		self.z_max = config.Z_MAX


		# Block move params
		self.block_dim = config.BLOCK_DIM
		self.r_lvl0 = config.R_LVL0
		self.z_lvl0 = config.Z_LVL0

		self.r_pick_init = config.R_STARTPICK_OFFSET
		self.theta_pick_init = config.THETA_STARTPICK_OFFSET
		self.z_pick_init = config.Z_STARTPICK_OFFSET
		self.r_pick_end = config.R_ENDPICK_OFFSET
		self.theta_pick_end = config.THETA_ENDPICK_OFFSET
		self.z_pick_end = config.Z_ENDPICK_OFFSET


	def move_to_home_pose(self):
		"""Move to "home" configuration. The resulting pose has correct orientation."""
		try:
			self.robot.movej(self.j_home, acc=0.5, vel=0.4, radius=0, wait=True, relative=False)
			pose = self.robot.getl()
			pose[0] = self.r_home*math.cos(self.phi+self.theta_home) + self.cyl_ox
			pose[1] = self.r_home*math.sin(self.phi+self.theta_home) + self.cyl_oy
			pose[2] = self.z_home
			self.robot.movel(pose, acc=0.15, vel=0.2, radius=0, wait=True, relative=False)
			self.current_cyl = (self.r_home, self.theta_home, self.z_home)
			if self.debug:
				print "Initial cylinder coords: %s" % str([('%.2f' % i) for i in self.current_cyl])
		except Exception, e:
			raise UR5Exception('Move to home configuration failed: %s' % str(e))


	def calibrate_cylinder_sys(self):
		"""Calibrate the reference cylinder coordinate system."""
		# Move to reference coordinate system base
		self.move_to_home_pose()
		# Set reference coordinate system parameters
		csys = math3d.Transform()
		csys.orient.rotate_zb(self.phi)
		self.robot.set_csys("csys", csys)
		self.trans_base = self.robot.get_transform()
		self.cyl_ox, self.cyl_oy = self.cyl_offset


	def set_cylinder_coords(self):
		p = self.robot.getl()
		x = p[0] - self.cyl_ox
		y = p[1] - self.cyl_oy
		z = p[2]
		r = (x**2 + y**2)**0.5
		theta = math.atan2(y, x)
		self.current_cyl = (r, theta, z)


	def cylinder2cartesian(self, r, theta, z):
		"""Translate table cylinder coordinates to reference cartesian coordinates."""
		trans = math3d.Transform(self.trans_base) #deep copy
		trans.orient.rotate_zb(theta)
		trans.pos = math3d.Vector(r*math.cos(theta) + self.cyl_ox,
								  r*math.sin(theta) + self.cyl_oy,
								  z)
		return trans.pose_vector.tolist()


	def exec_move(self, move, wait=False):
		"""Execute a linear move. move_list = pose + [acc, vel, radius]"""
		prog = "movel(p[{},{},{},{},{},{}], a={}, v={}, r={})".format(*move)
		self.robot.send_program(prog)
		if wait:
			self.robot.wait_for_move()


	def exec_moves(self, move_list, wait=False):
		"""Executes a list of linear moves. move_list = pose + [acc, vel, radius]"""
		header = "def myProg():\n"
		end = "end\n"
		template = "movel(p[{},{},{},{},{},{}], a={}, v={}, r={})\n"
		prog = header + ''.join([template.format(*m) for m in move_list]) + end
		self.robot.send_program(prog)
		if wait:
			self.robot.wait_for_move()


	def movec(self, r, theta, z, wait=False):
		"""Circular move - uses URScript built-in function.

		Unfortunately, this doesn't work very well: The tool orientation at the target pose 
		is not right. URScript specifies that orientation in pose_via is not considered, however,
		it SHOULD consider the orientation of the end pose.
		"""
		curr_theta = self.current_cyl[1]
		pose_via = self.cylinder2cartesian(r, curr_theta+(theta-curr_theta)/2, z)
		pose = self.cylinder2cartesian(r, theta, z)
		self.robot.movec(pose_via, pose, acc=self.acc, vel=self.vel, radius=0, wait=wait)


	def linearize_arc(self, r, theta, z, resolution=math.pi/41, blend=0.03):
		"""movec is unreliable and does not finish in the correct pose. We work around this
		by linearizing the arc into segments with length given in resolution, and use
		movels with blending.

		IMPORTANT: blending radius have to be smaller than the angular resolution! If not,
		the robot will not finish the last move, because it is within the target
		"""
		move_list = []
		step = resolution
		curr_r, curr_theta, curr_z = self.current_cyl
		dtheta = theta - curr_theta

		segments = max(int(round(abs(dtheta)/step)), 1)
		theta_incr = dtheta/segments
		r_incr = (r-curr_r)/segments
		z_incr = (z-curr_z)/segments

		for i in range(1, segments):
			pose = self.cylinder2cartesian(curr_r + i*r_incr, curr_theta + i*theta_incr, curr_z + i*z_incr)
			move_list.append(pose + [self.acc, self.vel, blend])
		move_list.append(self.cylinder2cartesian(r, theta, z) + [self.acc, self.vel, 0]) #dont  blend last
		return move_list


	def movec_hax(self, r, theta, z, wait=False):
		"""movec with linearized arc."""
		# manual acc/vel
		move_list = self.linearize_arc(r, theta, z)
		if self.debug:
			print "move list for movec_hax"
			for p in move_list:
				print [('%.3f' % i) for i in p]
		self.exec_moves(move_list, wait=wait)


	def move(self, vec, t=0):
		"""Move in vec direction."""
		dr, dtheta, dz = vec
		r, theta, z = self.current_cyl

		#if dr == 0 and dtheta == 0 and dz == 0:
		#	self.robot.stopl(acc=self.stop_acc)
		#	return

		if dtheta == 0: #linear move
			if dz == 0:
				move = self.cylinder2cartesian(r+dr, theta, z) + [self.acc, self.vel, 0]
			else:
				if dr == 0:
					move = self.cylinder2cartesian(r, theta, z+dz) + [self.acc, self.vel_z, 0]
				else:
					move = self.cylinder2cartesian(r+dr, theta, z+dz*(self.vel_z/self.vel)) + [self.acc, self.vel, 0]
			self.exec_move(move)

		else: #arc move
			dtheta = dtheta/r # angle of arc segment with length 1
			if dz == 0:
				moves = self.linearize_arc(r+dr, theta+dtheta, z)
			else:
				moves = self.linearize_arc(r+dr, theta+dtheta, z+dz*(self.vel_z/self.vel))
			self.exec_moves(moves)

		if t != 0:
			time.sleep(t)
			self.robot.stopl(acc=self.stop_acc)


	def update(self, vec, dt):
		"""Update movements based on vec and dt"""

		if self.force_mon:
			self.zforce = self.robot.get_tcp_force(wait=False)[2]
			if abs(self.zforce) > self.force_constraint:
				# move opposite to z force
				dz = -1*self.zforce/abs(self.zforce)
				self.move((0, 0, dz), t=self.t_force)
				self.set_cylinder_coords()
				return

		# move?
		self.set_cylinder_coords()
		dr, dtheta, dz = vec
		r, theta, z = self.current_cyl

		if sum(map(abs, vec)) == 0:
			if vec != self.prev_vec:
				self.robot.stopl(acc=self.stop_acc)
			self.prev_vec = (0,0,0)
			return

		# Projected position does not work. Fuck this, add empirical offsets instead
		#curr_d = (r**2 + (r*theta)**2 + z**2)**0.5
		#curr_v = abs(curr_d - self.prev_d) / dt
		#self.prev_v = curr_v
		#self.prev_d = curr_d
		
		# check projected constraints.
		rnext = r + dr*0.031
		thetanext = theta + dtheta*0.029
		znext = z + dz*0.009
		if (rnext < self.r_min and dr < 0) or (rnext > self.r_max and dr > 0):
			r = self.r_min if dr < 0 else self.r_max
			dr = 0
		if (thetanext < self.theta_min and dtheta < 0) or (thetanext > self.theta_max and dtheta > 0):
			theta = self.theta_min if dtheta < 0 else self.theta_max
			dtheta = 0
		if (znext < self.z_min and dz < 0) or (znext > self.z_max and dz > 0):
			z = self.z_min if dz < 0 else self.z_max
			dz = 0

		vec = (dr, dtheta, dz)
		self.current_cyl = (r, theta, z)
		if sum(map(abs, vec)) == 0:
			if vec != self.prev_vec:
				self.robot.stopl(acc=self.stop_acc)
			self.prev_vec = (0,0,0)
			return

		# change move
		if vec != self.prev_vec:
			if sum(map(abs, self.prev_vec)) == 0: #from still
				self.move(vec)
				self.prev_vec = vec
				self.t_ch = 0
			elif not self.t_ch: #from another command
				self.t_ch = time.time()
				self.robot.stopl(acc=self.stop_acc*2)
			elif time.time() - self.t_ch > self.t_chcmd:
				self.move(vec)
				self.prev_vec = vec
				self.t_ch = 0


	def is_looping(self, dt, threshold=0.001):
		"""Polls the robot for change in pose vector. Blocks for dt seconds."""
		pose0 = self.robot.getl()
		time.sleep(dt)
		pose1 = self.robot.getl()
		v = [pose1[i]-pose0[i] for i in range(len(pose0))]
		if self.debug:
			_sum = sum(map(abs, v))
			print "dt = %.2f, v = ds/dt = %s, |v| = %.3f" % (dt, str([('%.2f' % i) for i in v]), _sum)
		return sum(map(abs, v)) > threshold


	def blocklevel2move(self, r_lvl, theta, z_lvl, acc, vel, blend):
		r = self.r_lvl0 - r_lvl*self.block_dim
		z = self.z_lvl0 + z_lvl*self.block_dim
		move = self.cylinder2cartesian(r, theta, z) + [acc, vel, blend]
		return move


	def pick_block(self, r_lvl, theta, z_lvl, acc, vel):
		"""This function concatenates 3 moves:

		1. Move the tool in front of the block.
		2. Move the tool into the hole.
		3. Lift the block.

		Here, we assume that the tool is nearby the block. Bringing the tool towards the block is
		not the responsibility of this function.

		r_lvl starts at 0, which correspond to the outer edge of the table.
		z_lvl starts at 0, which corrsepond to table level.
		"""
		r = self.r_lvl0 - r_lvl*self.block_dim
		z = self.z_lvl0 + z_lvl*self.block_dim
		p1 = self.cylinder2cartesian(r+self.r_pick_init, theta+self.theta_pick_init, z+self.z_pick_init)
		p2 = self.cylinder2cartesian(r, theta, z)
		p3 = self.cylinder2cartesian(r+self.r_pick_end, theta+self.theta_pick_end, z+self.z_pick_end)
		move_list = [
			p1 + [self.acc, vel, 0.005],
			p2 + [self.acc, vel, 0],
			p3 + [self.acc, vel, 0.01],
		]
		return move_list


	def place_block(self, r_lvl, theta, z_lvl, acc, vel):
		"""Reverse move of pick_block."""
		moves = self.pick_block(r_lvl, theta, z_lvl, acc, vel)
		moves.reverse()
		return moves


	def construct_moves(self, task_list, acc, vel):
		"""Construct moves from a list of tasks"""
		pass # wait with implementing this


	def set_freedrive(self, mode):
		self.robot.set_freedrive(mode)

	def cleanup(self):
		self.robot.cleanup()
