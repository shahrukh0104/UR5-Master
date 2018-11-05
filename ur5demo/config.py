import math

DEBUG = True


# GUI
#
# Frame rates should be high enough to poll keys efficiently.
SCREEN_DIM = (800, 600)
FONTSIZE = 24
BG_COL = (255,255,255)
FONT_COL = (0,0,0)
MENU_FPS = 15
CTR_FPS = 60 #manual control



# NETWORK
#
# The IP address can be found in the PolyScope interface (tablet) of the robot.
# SETUP Robot -> Setup NETWORK (requires password: "ngr12") -> IP address
UR5_IP = "10.42.0.63"
UR5_HOSTNAME = 'ur-2012208984' #requires dns.



# BLOCK DIMENSION
#
# Blocks are cubic and the hole is centered.
BLOCK_DIM = 0.0965 #TUNED


# TABLE CONFIGURATION
#
# The table configuration specifies parameters for the cylinder coordinate system.
# All table parameters are relative to the robots base coordinate system.
#
# TABLE_QUADRANT was added because the UR5's kinetmatics computations are dependent
# on its own base coordinate system. For instance, quadrant 3.0 results in joint
# violations in the original setup. Quadrant 0.0 corresponds to the table center
# along the base x-axis, quadrant 1.0 corresponds to the base y-axis, etc.
# Quardant 0 is recommended.
TABLE_QUADRANT = 0 #[0..4)
TABLE_ORIGO_OFFSET = (-0.245, -0.003) #offset from origo along base x-axis
TABLE_Z = -0.3339 #this should be fine-tuned on each setup
TABLE_ARC = math.pi/2



# HOME JOINT CONFIGURATION
#
# This is the joint config for the robot's initial position.
# We need this also expressed in joint space to ensure that the robot does not
# collide with itself when moving to the base configuration, and to ensure
# the correct tool orientation when calibrating the reference cylinder
# coordinate system.
#
# Change joints 2-6 in QUADRANT_JOINTS_HOME to tune the tool orientation.
R_HOME = 0.55
THETA_HOME = 0
Z_HOME = TABLE_Z + BLOCK_DIM
JOINTS_HOME = [TABLE_QUADRANT*math.pi/2,
			   	   		-3*math.pi/4,
			   	   		-3*math.pi/4,
			   	   		-math.pi/2,
			   	  		-math.pi/2,
			  	  		49*math.pi/50]


# FORCE MONITOR
#
# Enables force constraints on the tool (for manual control).
# When the force exceeds FORCE_CONSTRAINT, the robot move in the direction
# opposite to the current command in T_FORCE_VIOLATION seconds.
# Note: Setting this to "True" involves polling the UR5 through a real-time
# monitor that runs at 125Hz. If the controller runs slow, try disabling this.
USE_FORCE_MONITOR = True
FORCE_CONSTRAINT = 40 # Newton
T_FORCE_VIOLATION = 0.3



# MANUAL CONTROL
#
# Parameters that specifies velocity curve and constraints of manual control.
# Note that the angular velocity is also [m/s], not [rad/s], to ensure that the robot
# moves with constant speed regardless of direction in the polar plane.
# IMPORTANT: VEL_Z should be tuned to Z_MIN to avoid smashing into the table.
#
# T_DIR_CHANGE_COOLDOWN is a cooldown between change of commands. With this, the robot
# have a chance to decelerate before changing direction, which smooths out motion.
VEL = 0.1 #[m/s]
VEL_Z = 0.05 #[m/s]
ACC = 0.5 #[m/s^2]
STOP_ACC = 1.0 #aka deceleration
T_DIR_CHANGE_COOLDOWN = 0.05

# Constraints are relative to the cylinder coordinate system.
# Because of delay and fluctuations of parameters in the system,
# the calculation of projected positions are not correct.
# Therefore, an empirical offset is added in controller.update().
# Consider this before changing control speed
R_MIN = 0.55
R_MAX = 0.75
THETA_MIN= -TABLE_ARC/2
THETA_MAX = TABLE_ARC/2
Z_MIN = TABLE_Z + 0.5*BLOCK_DIM - 0.005
Z_MAX = TABLE_Z + 4.5*BLOCK_DIM



# GAMEPAD MAP
#
# For the Microsoft xBox 360 controller. Button/axes codes are listed
# below for reference.
#
# Buttons
# A:0
# B:1
# X:2
# Y:3
# LB:4
# RB:5
# Back:6
# Start:7
#
# Axes (Not supported)
# Joystick left, x-axis:0
# Joystick left, y-axis:1
# Joystick right, x-axis:3
# Joystick right, x-axis:4
# Left trigger:2
# Right trigger:5
#
# Hat switch:0 (All or nothing for direction). returns (+-1, +-1) = (+-x. +-y)
# hx, hy = hat
HATCODE = -1 #set to -1 to avoid polling hat
HX = 10
HY = 11
HAT_MAP = {
	HX: (0,1,0),
	HY: (-1,0,0) #inverted
}

#BUTTON_MAP = {
#	4: (0,0,-1),
#	5: (0,0,1)
#}
BUTTON_MAP = { #Buttons only (no hat)
	0: (1,0,0),
	3: (-1,0,0),
	1: (0,1,0),
	2: (0,-1,0),
	4: (0,0,-1),
	5: (0,0,1)
}


# AUTO/LOOP CONTROL
#
# When looping, the robot use the same velocity for r, theta and z.
# Also, since this is all planned motions, we dont need to use stopl().
# This means that the trapezoidal velocity curve has the same slope at
# start and end of a motion. (acceleration=deceleration)
#
# Constraints in r and z-direction are specefied in levels to enable
# easier construction of loops. Note that the r-levels grow inwards, and
# the z-levels grow upwards.
DEFAULT_LOOP_VEL = 0.1
LOOP_VEL_MIN = 0.05
LOOP_VEL_MAX = 1.5
LOOP_ACC = 0.5

R_LVL0 = 0.691 #tip of tool at table edge
Z_LVL0 = TABLE_Z + BLOCK_DIM/2 #tool in middle of block
THETA_EDGE_LEFT = -math.pi/4 + math.pi/74 #empirical. table is not exact
THETA_EDGE_RIGHT = math.pi/4 - math.pi/74

# Offsets when picking (and placing) blocks.
# The place-block move is the reverse of the pick-block move,
# such that the start/end offsets for pick-block are the end/start offsets
# for place-block
R_STARTPICK_OFFSET = -1.3*BLOCK_DIM
THETA_STARTPICK_OFFSET = 0
Z_STARTPICK_OFFSET = 0
R_ENDPICK_OFFSET = -0.01 #lift slightly inwards to avoid collision with adjacent blocks
THETA_ENDPICK_OFFSET = 0
Z_ENDPICK_OFFSET = BLOCK_DIM*0.5

