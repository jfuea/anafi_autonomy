#!/usr/bin/env python3
import sys
import threading
import cv2
import queue

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import olympe

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


class AnafiVideo(threading.Thread):

    def __init__(self):
        # *param server
        DRONE_IP = rospy.get_param("DRONE_IP", "10.202.0.1")
        # general
        self.drone = olympe.Drone(DRONE_IP)
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        super().__init__()
        super().start()
        self.bridge = CvBridge()
        # topics
        self.camera_pub = rospy.Publisher(
            "camera/image_raw", Image, queue_size=1)

    def start(self):
        self.drone.connect()
        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()

    def stop(self):
        self.drone.disconnect()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def flush_cb(self):
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def run(self):
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.publish_frame_content(yuv_frame)
                except Exception:
                    # We have to continue popping frame from the queue even if
                    # we fail to show one frame
                    traceback.print_exc()
                finally:
                    # Don't forget to unref the yuv frame. We don't want to
                    # starve the video buffer pool
                    yuv_frame.unref()

    def publish_frame_content(self, yuv_frame):
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


def main(args):
    video = AnafiVideo()
    rospy.init_node('anafi_video', anonymous=True)
    video.start()
    while not rospy.is_shutdown():
        rospy.spin()
    video.stop()


if __name__ == '__main__':
    main(sys.argv)
