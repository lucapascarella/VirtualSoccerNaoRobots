import os
import time

from resources.motions import Motions

class Player:
    def __init__(self):
        self.request_queue = None
        self.frame_queue = None
        self.sensor_queue = None

        self.frame_top = None
        self.frame_bottom = None

    def setQueues(self, request_queue, frame_queue, sensor_queue):
        self.request_queue = request_queue
        self.frame_queue = frame_queue
        self.sensor_queue = sensor_queue

    def getImu(self):
        """
        Returns Inertial Measurement Unit (IMU) value
        (angular_velocity, linear_acceleration)
            angular_velocity - The angular velocity values in rad/s
            linear_acceleration - The linear acceleration values in m/s^2
        """
        self.request_queue.put(["sensor", "imu"])
        return self.sensor_queue.get()

    def getFsr(self):
        """
        Returns Force Sensitive Resistor (FSR) values for left and right feet
        ([LFOOT_FL, LFOOT_FR, LFOOT_RL, LFOOT_RR], [RFOOT_FL, RFOOT_FR, RFOOT_RL, RFOOT_RR])
            LFOOT_FL - Left foot, Front left
            LFOOT_FR - Left foot, Front right
            LFOOT_RL - Left foot, Rear left
            LFOOT_RR - Left foot, Rear right
            RFOOT_FL - Right foot, Front left
            RFOOT_FR - Right foot, Front right
            RFOOT_RL - Right foot, Rear left
            RFOOT_RR - Right foot, Rear right
        """
        self.request_queue.put(["sensor", "fsr"])
        return self.sensor_queue.get()

    def getCameraFrame(self, position):
        """
        Parameter:
            position - target camera in string

        Available cameras: top or bottom

        Returns camera image frame
        """
        assert(position == "top" or position == "bottom")
        self.request_queue.put(["camera", position])
        return self.frame_queue.get()

    def setAngles(self, joint_dict):
        """
        Parameter:
            joint_dict - dictionary of joint value (key: joint name)

        Available joints: HeadYaw, HeadPitch, LHipYawPitch, LHipRoll, 
                          LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll,
                          RHipYawPitch, RHipRoll, RHipPitch, RKneePitch,
                          RAnklePitch, RAnkleRoll, LShoulderPitch, LShoulderRoll,
                          LElbowYaw, LElbowRoll, LWristYaw, LHand, LFinger21, LFinger22, 
                          LFinger23, LFinger11, LFinger12, LFinger13, LThumb1, LThumb2,
                          RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, 
                          RWristYaw, RHand, RFinger21, RFinger22, RFinger23, 
                          RFinger11, RFinger12, RFinger13, RThumb1, RThumb2
        """
        self.request_queue.put(["move", joint_dict])
        return

    def pose(self, posture, speed=1.0):
        """
        Parameters:
            posture - target position in string
            speed - speed of posture

        Available postures: Stand, StandInit, StandZero, Crouch, Sit,
                            SitRelax, LyingBelly, LyingBack
        """
        assert(speed > 0.0 and speed <= 1.0)
        self.request_queue.put(["pose", (posture, speed)])
        return

    def move(self, motion):
        """
        Parameter:
            motion - target motion in string

        Available motions: Backwards, Forwards, Shoot, SideStepLeft,
                           SideStepRight, StandUpFromBack, TurnLeft, TurnRight
        """
        poses = Motions.getMotion(motion)

        for i, (timing, pose) in enumerate(poses):
            self.request_queue.put(["move", pose])
            time.sleep(0.1) # default delay

        return

    def behave(self):
        raise NotImplementedError
