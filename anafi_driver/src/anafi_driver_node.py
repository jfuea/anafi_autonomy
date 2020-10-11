#!/usr/bin/env python3
import sys
from math import pi

import rospy
from geometry_msgs.msg import Twist

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.ardrone3.SpeedSettingsState import (
    MaxPitchRollRotationSpeedChanged, MaxRotationSpeedChanged,
    MaxVerticalSpeedChanged
)

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


class AnafiControl:

    def __init__(self):
        # *param server
        DRONE_IP = rospy.get_param("DRONE_IP", "10.202.0.1")
        # general
        self.drone = olympe.Drone(DRONE_IP)
        # SpeedSettings
        self.speed_settings = False
        self.max_vertical_speed = None
        self.max_pitch_roll = None
        self.max_yaw = None
        # conversions
        # TODO may be a function, need more info
        self.tilt_to_speed = 6
        # topics
        self.sub_cmd_vel = rospy.Subscriber(
            "cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)

    def start(self):
        self.drone.connect()
        self.get_speed_settings()
        self.drone.start_piloting()

    def get_speed_settings(self):
        """
        Get current maximum speed settings, olympe work with degrees and m/s
        """
        vertical = self.drone.get_state(MaxVerticalSpeedChanged)
        self.max_vertical_speed = vertical['current']
        pitch_roll = self.drone.get_state(MaxPitchRollRotationSpeedChanged)
        self.max_pitch_roll = pitch_roll['current'] * pi / 180
        yaw = self.drone.get_state(MaxRotationSpeedChanged)
        self.max_yaw = yaw['current'] * pi / 180
        self.speed_settings = True

    def stop(self):
        self.drone.disconnect()

    def take_off(self):
        """Try to takeoff if is not already hovering or moving"""
        if (self.drone.get_state(FlyingStateChanged)["state"] is not
                FlyingStateChanged_State.hovering):
            self.drone(TakeOff(_no_expect=True)
                       & FlyingStateChanged(state="hovering", _policy="wait",
                                            _timeout=5)
                       ).wait()

    def landing(self):
        self.drone.stop_piloting()
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
        )

    # callbacks
    def cmd_vel_cb(self, twist_data):
        if self.speed_settings:
            # TODO compare twists, threading, safe division when None max sett
            xy_conversion = self.tilt_to_speed * 100 // self.max_pitch_roll
            pitch = int(twist_data.linear.x * xy_conversion)
            roll = int(twist_data.linear.y * xy_conversion)
            throttle = int(
                twist_data.linear.z * 100 // self.max_vertical_speed)
            yaw = - int(twist_data.angular.z * 100 // self.max_yaw)
            rospy.logerr("{}, {}, {}, {}".format(pitch, roll, throttle, yaw))
            # send every 50ms
            self.drone.piloting_pcmd(roll, pitch, yaw, throttle, 0)


def main(args):
    ctrl = AnafiControl()
    rospy.init_node('anafi_driver', anonymous=True)
    ctrl.start()
    ctrl.take_off()
    while not rospy.is_shutdown():
        rospy.spin()
    ctrl.landing()
    ctrl.stop()


if __name__ == '__main__':
    main(sys.argv)
