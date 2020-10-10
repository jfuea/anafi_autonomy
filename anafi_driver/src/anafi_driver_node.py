#!/usr/bin/env python3
import sys
import rospy
import olympe


class Control:

    def __init__(self):
        DRONE_IP = rospy.get_param("DRONE_IP", "10.202.0.1")
        self.drone = olympe.Drone(DRONE_IP)
        self.drone.start_piloting()

    def start(self):
        self.drone.connect()

    def stop(self):
        self.drone.stop_piloting()
        self.drone.disconnect()


def main(args):
    ctrl = Control()
    ctrl.start()
    while not rospy.is_shutdown():
        rospy.spin()
    ctrl.stop()


if __name__ == '__main__':
    main(sys.argv)
