from Robot.UR.URRobot import URRobot
import time
import signal
import sys

class TrajectoryManager():
    def __init__(self, robot=URRobot("host")):
        self.robot = robot
        self.movel_speed = 0.1
        self.movej_speed = 0.1
        self.ok_to_execute = True
        # Configurer le gestionnaire de signal pour SIGINT
        signal.signal(signal.SIGINT, self.handler)

    def set_movel_speed(self, movel_speed):
        self.movel_speed = movel_speed

    def set_movel_poses(self, movel_poses):
        self.movel_poses = movel_poses

    def set_movej_speed(self, movej_speed):
        self.movej_speed = movej_speed

    def set_movej_poses(self, movej_poses):
        self.movej_poses = movej_poses

    def play_trajectory_movel(self, loop=1):
        for i in range(loop):
            for i, movel_pose in enumerate(self.movel_poses) :
                if self.ok_to_execute:
                    # Set starting position
                    print("reaching movel_pose {}".format(i))
                    # command_sent = robot.movel((0.3, -0.5, 0.2, 0, 3.14, 0), v=0.1)
                    command_sent = self.robot.movel(movel_pose, v=self.movel_speed)
                    while not command_sent:
                        time.sleep(0.1)
                    reached = False
                    positions = [None]
                    while not reached:
                        position = self.robot.get_tcp_position()
                        print("position = ", position)
                        time.sleep(0.2)
                        if position == positions[-1]:
                            reached = True
                        else :
                            positions.append(position)
                    print("movel_pose {} reached".format(i))

    def play_trajectory_movej(self, loop=1):
        for nloop in range(loop):
            for i, movej_pose in enumerate(self.movej_poses) :
                if self.ok_to_execute:
                    print("reaching movej_pose {}".format(i))
                    # command_sent = robot.movel((0.3, -0.5, 0.2, 0, 3.14, 0), v=0.1)
                    command_sent = self.robot.movej(movej_pose, v=self.movej_speed)
                    while not command_sent:
                        time.sleep(0.1)
                    self.wait_position_reached()
                    print("movel_pose {} reached".format(i))
                    # time.sleep(10)
            print("loop number {}".format(nloop))

    def wait_position_reached(self):
        reached = False
        positions = [None]
        while not reached:
            position = self.robot.get_tcp_position()
            print("position = ", position)
            time.sleep(0.3)
            if position == positions[-1]:
                reached = True
            else :
                positions.append(position)

    def stop_execution(self):
        self.ok_to_execute = False
        self.robot.stopl()
        self.robot.stopj()

    def handler(self, signal, frame):
        # Code à exécuter lorsqu'un signal SIGINT est reçu
        print("Signal SIGINT reçu. Arrêt en cours...")
        self.stop_execution()
        # sys.exit(0)

