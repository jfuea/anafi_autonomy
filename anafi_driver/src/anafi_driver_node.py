#!/usr/bin/env python3
import sys
from math import pi
import threading
import cv2
import queue

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.ardrone3.SpeedSettingsState import (
    MaxPitchRollRotationSpeedChanged, MaxRotationSpeedChanged,
    MaxVerticalSpeedChanged
)

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


class Anafi():

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
        # video
        self.bridge = CvBridge()
        # conversions
        # TODO may be a function, need more info
        self.tilt_to_speed = 6
        # topics
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb, queue_size=1)
        rospy.Subscriber("takeoff", Empty, self.take_off, queue_size=1)
        rospy.Subscriber("land", Empty, self.land, queue_size=1)
        self.camera_pub = rospy.Publisher(
            "camera/image_raw", Image, queue_size=1)

    def start(self):
        self.drone.connect()
        self.get_speed_settings()
        self.drone.start_piloting()
        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()

    def stop(self):
        self.drone.disconnect()

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

    def take_off(self, data):
        """Try to takeoff if is not already hovering or moving"""
        if (self.drone.get_state(FlyingStateChanged)["state"] is not
                FlyingStateChanged_State.hovering):
            self.drone(TakeOff(_no_expect=True)
                       & FlyingStateChanged(state="hovering", _policy="wait",
                                            _timeout=2)
                       ).wait()

    def land(self, data):
        self.drone.stop_piloting()
        if (self.drone.get_state(FlyingStateChanged)["state"] is not
                FlyingStateChanged_State.landed):
            self.drone(Landing(_no_expect=True)
                       & FlyingStateChanged(state="landed", _policy="wait",
                                            _timeout=2)
                       ).wait()

    # video
    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        # self.publish_frame_content(yuv_frame)
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # yuv_frame.vmeta() returns a dictionary that contains additional
        # metadata from the drone (GPS coordinates, battery percentage, ...)

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)
        try:
            self.camera_pub.publish(
                # self.bridge.cv2_to_imgmsg(cv2frame, "bgr16"))
                self.bridge.cv2_to_imgmsg(cv2frame))
        except CvBridgeError as e:
            print(e)

    # callbacks
    def cmd_vel_cb(self, twist_data):
        if self.speed_settings:
            # TODO compare twists, threading, safe division when None max sett
            # TODO signs
            xy_conversion = self.tilt_to_speed * 100 // self.max_pitch_roll
            pitch = - int(twist_data.linear.x * xy_conversion)
            roll = int(twist_data.linear.y * xy_conversion)
            throttle = int(
                twist_data.linear.z * 100 // self.max_vertical_speed)
            yaw = - int(twist_data.angular.z * 100 // self.max_yaw)
            # run every 50ms in async mode for every pkg unless changed
            self.drone.piloting_pcmd(roll, pitch, yaw, throttle, 0)


def main(args):
    ctrl = Anafi()
    rospy.init_node('anafi_driver', anonymous=True)
    ctrl.start()
    while not rospy.is_shutdown():
        rospy.spin()
    ctrl.stop()


if __name__ == '__main__':
    main(sys.argv)
