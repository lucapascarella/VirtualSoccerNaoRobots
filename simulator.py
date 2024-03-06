#!/usr/bin/env python

import math
import traceback
import time
import pybullet as p
import pybullet_data as pd

from qibullet import SimulationManager
from qibullet import NaoVirtual, NaoFsr
from qibullet import Camera

import multiprocessing as mp

from teams.simple_team import SimpleTeam

# Number of players per team
NUM_PLAYERS = 1

# Player starting position
TEAM_A = -0.1
TEAM_B = 1


class RoboCupSimulator:
    def __init__(self, teamClassA, teamClassB):
        # Launch simulation
        self.manager = SimulationManager()
        self.client = self.manager.launchSimulation(gui=True, auto_step=False)

        # Set OpenGL visualizer
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, rgbBackground=(0, 0, 0), physicsClientId=self.client)

        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

        p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=180, cameraPitch=230, cameraTargetPosition=[0, 0, 0])

        # Spawn soccor field
        p.setAdditionalSearchPath("./resources/objects")

        self.field = p.loadSDF("soccerfield.sdf", globalScaling=0.1, physicsClientId=self.client)

        # p.loadURDF(
        #    "mark_yellow.urdf",
        #    basePosition=[-2.6, 0, 0],
        #    globalScaling=1.0,
        #    physicsClientId=self.client)

        # p.loadURDF(
        #    "mark_cyan.urdf",
        #    basePosition=[2.6, 0, 0],
        #    globalScaling=1.0,
        #    physicsClientId=self.client)

        self.teamClassA = teamClassA
        self.teamClassB = teamClassB

        # Create teams
        self.teamA = self.teamClassA(NUM_PLAYERS, TEAM_A)
        self.teamB = self.teamClassB(NUM_PLAYERS, TEAM_B)

        self.goals = [(TEAM_B, [-1.76, 0.4, 0.48]), (TEAM_A, [1.76, 0.4, 0.48])]

        # Store a list of robots
        self.robots = [None] * (NUM_PLAYERS * 2)

    def reset(self):
        self.ball = p.loadURDF("soccerball.urdf", basePosition=[0, 0, 0], globalScaling=0.15, physicsClientId=self.client)

        p.changeDynamics(self.ball, -1, lateralFriction=0.1, spinningFriction=0.1, rollingFriction=0.1)

        # Get a string version of the class name
        nameA = self.teamClassA.__name__.rjust(15)
        nameB = self.teamClassB.__name__.ljust(15)

        # Prepare leaderboard scores string
        name_str = nameA + " " + str(self.teamA.score)
        name_str += " vs "
        name_str += str(self.teamB.score) + " " + nameB

        # Show leaderboard in form of debug text
        self.score = p.addUserDebugText(name_str, [-1.55, 3.0, 0.1], textOrientation=[0.5, 0, 0, 1], textColorRGB=[1, 1, 1], textSize=0.25)

        for i in range(NUM_PLAYERS):
            posA = self.teamA.getFormation(i)
            posB = self.teamB.getFormation(i)

            posA = [(1.6 - posA[0]) * TEAM_A, posA[1] * TEAM_A, 0]
            posB = [(1.6 - posB[0]) * TEAM_B, posB[1] * TEAM_B, 0]

            robotA = Robot(self.manager.spawnNao(self.client, posA, [0, 0, 0, 1], spawn_ground_plane=False))

            robotB = Robot(self.manager.spawnNao(self.client, posB, [0, 0, 1, 0], spawn_ground_plane=False))

            self.teamA.setPlayer(i)
            self.teamB.setPlayer(i)

            self.teamA.players[i].setQueues(robotA.request_queue, robotA.frame_queue, robotA.sensor_queue)
            self.teamB.players[i].setQueues(robotB.request_queue, robotB.frame_queue, robotB.sensor_queue)

            self.robots[i * 2] = robotA
            self.robots[i * 2 + 1] = robotB

    def loop(self):
        try:
            self.teamA.play()
            self.teamB.play()

            prePos = p.getBasePositionAndOrientation(self.ball)[0]

            while True:
                # Handle Robot's requests iteratively
                for robot in self.robots:
                    if not robot.request_queue.empty():
                        request = robot.request_queue.get()
                        robot.handleRequest(request)

                curPos = p.getBasePositionAndOrientation(self.ball)[0]
                result = self.checkGoal(curPos, prePos)

                if result == TEAM_A:
                    self.teamA.score += 1
                    break
                elif result == TEAM_B:
                    self.teamB.score += 1
                    break

                prePos = curPos

                self.manager.stepSimulation(self.client)
                time.sleep(1. / 240.)

            self.printResult()

        except Exception as e:
            print(traceback.format_exc())

            self.teamA.stop()
            self.teamB.stop()

            self.manager.stopSimulation(self.client)

    def checkGoal(self, cur_pos, pre_pos):
        if cur_pos[0] == pre_pos[0]:
            return 0

        for (team_id, team_goal) in self.goals:
            if team_goal[0] < 0 and (pre_pos[0] < team_goal[0] or cur_pos[0] > team_goal[0]):
                continue

            if team_goal[0] > 0 and (pre_pos[0] > team_goal[0] or cur_pos[0] < team_goal[0]):
                continue

            t = (cur_pos[0] - pre_pos[0]) / (team_goal[0] - pre_pos[0])
            y = (cur_pos[1] - pre_pos[1]) / t + cur_pos[1]
            z = (cur_pos[2] - pre_pos[2]) / t + cur_pos[2]

            if team_goal[1] >= y >= - team_goal[1] and z <= team_goal[2]:
                return team_id

        return 0

    def printResult(self):
        if self.teamA.score > self.teamB.score:
            winner = self.teamClassA.__name__
        elif self.teamA.score < self.teamB.score:
            winner = self.teamClassB.__name__
        else:
            winner = 'no one! ;)'

        result_str = winner + ' is the winner!'
        p.addUserDebugText(result_str.center(32), [-1.35, 0.0, 0.25], textOrientation=[0.5, 0, 0, 1], textColorRGB=[1, 1, 1], textSize=0.25)


class Robot:
    def __init__(self, object):
        self.object = object
        self.request_queue = mp.Queue()
        self.frame_queue = mp.Queue()
        self.sensor_queue = mp.Queue()

        self.link_top = self.object.link_dict["CameraTop_optical_frame"]
        self.link_bottom = self.object.link_dict["CameraBottom_optical_frame"]

        self.hfov = 60.9
        self.vfov = 47.6

        self.near_plane = 0.01
        self.far_plane = 100.

    def handleRequest(self, request):
        if request is None:
            return

        request_type = request[0]
        value = request[1]

        if request_type == "pose":
            self.object.goToPosture(value[0], value[1])

        elif request_type == "move":
            joint_names = list(value.keys())
            joint_values = list(value.values())
            self.object.setAngles(joint_names, joint_values, 1.0)

        elif request_type == "camera":
            if value == "top":
                self.frame_queue.put(self.getCameraFrame(self.link_top))
            elif value == "bottom":
                self.frame_queue.put(self.getCameraFrame(self.link_bottom))

        elif request_type == "sensor":
            if value == "imu":
                self.sensor_queue.put(self.object.getImuValues())
            elif value == "fsr":
                left_values = self.object.getFsrValues(NaoFsr.LFOOT)
                right_values = self.object.getFsrValues(NaoFsr.RFOOT)
                self.sensor_queue.put((left_values, right_values))

    def getCameraFrame(self, link):
        _, _, _, _, pos_world, q_world = p.getLinkState(self.object.robot_model, link.getParentIndex(), computeForwardKinematics=False)

        rotation = p.getMatrixFromQuaternion(q_world)
        forward_vector = [rotation[0], rotation[3], rotation[6]]
        up_vector = [rotation[2], rotation[5], rotation[8]]

        camera_target = [
            pos_world[0] + forward_vector[0] * 10,
            pos_world[1] + forward_vector[1] * 10,
            pos_world[2] + forward_vector[2] * 10]

        view_matrix = p.computeViewMatrix(pos_world, camera_target, up_vector)

        projection_matrix = p.computeProjectionMatrix(
            left=-math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
            right=math.tan(math.pi * self.hfov / 360.0) * self.near_plane,
            bottom=-math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
            top=math.tan(math.pi * self.vfov / 360.0) * self.near_plane,
            nearVal=self.near_plane,
            farVal=self.far_plane)

        frame = p.getCameraImage(160, 160, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL, flags=p.ER_NO_SEGMENTATION_MASK)[2]

        return frame

    def getImu(self):
        linear, angular = p.getBaseVelocity(self.object.robot_model)
        return (angular, linear)


def main():
    simulator = RoboCupSimulator(SimpleTeam, SimpleTeam)
    simulator.reset()
    simulator.loop()


if __name__ == "__main__":
    main()
