import numpy as np
import time
from math import pi

from Robot.UR.URRobot import URRobot
from Robot.UR.URTrajectory import TrajectoryManager

host = "192.168.1.100"
robot = URRobot(host)
robot.connect()

# Set TCP offset
robot.set_tcp((0.0, 0.0, 0.0, 0, 0, 0))
time.sleep(0.5)

movel_poses = [
	(0.3, -0.5, 0.2, 0, 3.14, 0),
	(-0.3, -1.0, 0.2, 0, 3.14, 0)
]

movej_poses = [
    (0, -pi/2, 0, -pi/2, 0, 0),
    (0.50790014, -2.22490952,  4.33187251, -1.19016608, -2.4005959,   0.),
    (0, -pi/2, 0, -pi/2, 0, 0),
    (2.77801011, -0.94678942,  1.9178288,  -1.41110922,  2.7434806,   0.)
	]

movej_poses = [
    (0, -pi/2, 0, -pi/2, 0, 0),
	(  1.65148757, -0.60650281,  1.48183948, 13.38819423, -0.57048845,  0.        ),
    (0, -pi/2, 0, -pi/2, 0, 0),
     (4.79094956, -2.42225468, -1.58034144,  2.87007202,  0.18341728,  0.)        
]

# view _1
home = (0, -pi/2, 0, -pi/2, 0, 0)
pose_gauche =  (0.91238218, -1.1488085,  -4.58578578, -1.52971036,  2.49609305,  3.83885372)
pose_droite =   (-1.26344462, -2.75290419, -0.62711242, -5.52078564, -4.67626609, -3.37189831)

movej_poses = [home, pose_gauche] 
movej_poses = [home, pose_droite]

# view _2
home = (0, -pi/2, 0, -pi/2, 0, 0)
pose_gauche =  (-5.37080316, -1.14880847,  1.69739941, -1.52971018, -3.78709227, -2.4443315 )
pose_droite =   (-4.71048956, -0.7530719,   0.97273887, -0.7703025,  -5.01148014, -3.80925941)

movej_poses = [home, pose_gauche] 

# movej_poses = [home, pose_droite, home, pose_gauche]

import math

def remap_j_poses(movej_poses_to_remap):
	for i, pose in enumerate(movej_poses_to_remap):
		results = []
		for a in pose:
			radians = a

			result = math.fmod(radians, 2*math.pi)

			if result > math.pi:
				result -= 2*math.pi
			elif result < -math.pi:
				result += 2*math.pi
			results.append(result)
		movej_poses_to_remap[i] = tuple(results)
	return	movej_poses_to_remap

# remmaped example : 
# [(0.0, -1.5707963267948966, 0.0, -1.5707963267948966, 0.0, 0.0), 
#  (1.5726957471795862, -0.7530719, 0.97273887, -0.7703025, 1.2717051671795865, 2.473925897179586)]
pose_gauche =  (-1.8411567353119456, -2.34979994082731, -1.2266999424328147, 1.9908767037689437, 0.5236622435773014, 3.1662042218688393)
pose_droite = (-1.263444673106254, -2.752904714500044, -0.6271113512074903, 0.7623991415576173, 1.606919278944055, 1.5916625032420637)


trajman = TrajectoryManager(robot)
# trajman.set_movel_poses(movel_poses)
# trajman.set_movel_speed(0.1)
# trajman.play_trajectory_movel(loop=2)


movej_poses = [home, pose_gauche] 
# movej_poses_remapped = remap_j_poses(movej_poses)

trajman.set_movej_poses(movej_poses)
trajman.set_movej_speed(0.15)
trajman.play_trajectory_movej(loop=1)


vec_gauche = (-0.00995, -0.00962, 0.01834)
vec_droite = (-0.00282, -0.00325, 0.04361)

robot.translate(vec_gauche)
time.sleep(5)

movej_poses = [home, pose_droite]
# # movej_poses_remapped = remap_j_poses(movej_poses)

trajman.set_movej_poses(movej_poses)
trajman.set_movej_speed(0.15)
trajman.play_trajectory_movej(loop=1)
robot.translate(vec_droite)
