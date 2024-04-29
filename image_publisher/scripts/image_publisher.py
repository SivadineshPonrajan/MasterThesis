#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import numpy as np
import cv2
import open3d
import time
import argparse
import os
import sys
import queue
import glob
from abc import ABCMeta, abstractmethod  # Abstract class.

ROOT = os.path.dirname(os.path.abspath(__file__))+'/'

FRAME_ID = "head_camera"  # Name of the camera frame.

IMAGE_SUFFIXES = ["png", "jpg"]  # What kind of image to read from folder.

APPEND_RANDOM_NUMBER_TO_NODE_NAME = True  # Make node name unique.


def parse_command_line_arguments():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description="Publish color or depth images from a folder to ROS topic.")
    parser.add_argument("-i", "--images_folder", required=True, type=str,
                        default=ROOT+"data/color/",
                        help="A data folder which contains .jpg or .png images for publishing.")
    parser.add_argument("-t", "--topic_name", required=False, type=str,
                        default="test_data/image_raw",
                        help="ROS topic name to publish image.")
    parser.add_argument("-r", "--publish_rate", required=False, type=float,
                        default=1.0,
                        help="How many images to publish per second.")
    parser.add_argument("-f", "--format", required=False, type=str,
                        choices=["color", "depth"],
                        default="color",
                        help="Format of image: color (uint8, 3 channels), or depth (uint16, 1 channel).")
    args = parser.parse_args(rospy.myargv()[1:])
    return args


def create_header(frame_id):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return header


def get_filenames(folder, suffixes=IMAGE_SUFFIXES):
    filenames = []
    for suffix in suffixes:
        filenames.extend(glob.glob(folder + "/*." + suffix))
    return sorted(filenames)


class AbstractImagePublisher(object):
    __metaclass__ = ABCMeta

    def __init__(self, image_topic,
                 queue_size=10):
        self._pub = rospy.Publisher(image_topic, Image, queue_size=queue_size)
        self._cv_bridge = CvBridge()

    def publish(self, image, frame_id="head_camera"):
        ros_image = self._to_ros_image(image)
        ros_image.header = create_header(frame_id)
        self._pub.publish(ros_image)

    @abstractmethod
    def _to_ros_image(self, image):
        pass


class ColorImagePublisher(AbstractImagePublisher):

    def __init__(self, topic_name, queue_size=10):
        super(ColorImagePublisher, self).__init__(
            topic_name, queue_size)

    def _to_ros_image(self, cv2_uint8_image, img_format="bgr"):

        # -- Check input.
        shape = cv2_uint8_image.shape  # (row, col, depth=3)
        assert(len(shape) == 3 and shape[2] == 3)

        # -- Convert image to bgr format.
        if img_format == "rgb":  # If rgb, convert to bgr.
            bgr_image = cv2.cvtColor(cv2_uint8_image, cv2.COLOR_RGB2BGR)
        elif img_format == "bgr":
            bgr_image = cv2_uint8_image
        else:
            raise RuntimeError("Wrong image format: " + img_format)

        # -- Convert to ROS format.
        ros_image = self._cv_bridge.cv2_to_imgmsg(bgr_image, "bgr8")
        return ros_image


class DepthImagePublisher(AbstractImagePublisher):

    def __init__(self, topic_name, queue_size=10):
        super(DepthImagePublisher, self).__init__(
            topic_name, queue_size)

    def _to_ros_image(self, cv2_uint16_image):

         # -- Check input.
        shape = cv2_uint16_image.shape  # (row, col)
        assert(len(shape) == 2)
        assert(type(cv2_uint16_image[0, 0] == np.uint16))

        # -- Convert to ROS format.
        ros_image = self._cv_bridge.cv2_to_imgmsg(cv2_uint16_image, "16UC1")
        return ros_image


class AbstractImageSubscriber(object):
    __metaclass__ = ABCMeta

    def __init__(self, topic_name, queue_size=2):
        self._cv_bridge = CvBridge()
        self._sub = rospy.Subscriber(
            topic_name, Image, self._callback_of_image_subscriber)
        self._imgs_queue = queue.Queue(maxsize=queue_size)

    @abstractmethod
    def _convert_ros_image_to_desired_image_format(self, ros_image):
        pass

    def get_image(self):
        ''' Get the next image subscribed from ROS topic,
            convert to desired opencv format, and then return. '''
        if not self.has_image():
            raise RuntimeError("Failed to get_image().")
        ros_image = self._imgs_queue.get(timeout=0.05)
        dst_image = self._convert_ros_image_to_desired_image_format(ros_image)
        return dst_image

    def has_image(self):
        return self._imgs_queue.qsize() > 0

    def _callback_of_image_subscriber(self, ros_image):
        ''' Save the received image into queue.
        '''
        if self._imgs_queue.full():  # If queue is full, pop one.
            img_to_discard = self._imgs_queue.get(timeout=0.001)
        self._imgs_queue.put(ros_image, timeout=0.001)  # Push image to queue


class ColorImageSubscriber(AbstractImageSubscriber):
    ''' RGB image subscriber. '''

    def __init__(self, topic_name, queue_size=2):
        super(ColorImageSubscriber, self).__init__(topic_name, queue_size)

    def _convert_ros_image_to_desired_image_format(self, ros_image):
        ''' To np.ndarray np.uint8 BGR format. '''
        return self._cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")


class DepthImageSubscriber(AbstractImageSubscriber):
    ''' Depth image subscriber. '''

    def __init__(self, topic_name, queue_size=2):
        super(DepthImageSubscriber, self).__init__(topic_name, queue_size)

    def _convert_ros_image_to_desired_image_format(self, ros_image):
        ''' To np.ndarray np.uint16 format. '''
        return self._cv_bridge.imgmsg_to_cv2(ros_image, "16UC1")  # not 32FC1


def main(args):

    # -- Read images filenames.
    images_filenames = get_filenames(
        args.images_folder, suffixes=IMAGE_SUFFIXES)
    if len(images_filenames) == 0:
        raise RuntimeError("Image folder has no image. "
                           "Check this folder again: " + args.images_folder)
    rospy.loginfo("There are {} images in: {}".format(
        len(images_filenames), args.images_folder))

    # -- Create image publisher.
    img_topic_name = args.topic_name
    if args.format == "color":
        img_publisher = ColorImagePublisher(img_topic_name)
    else:  # "depth"
        img_publisher = DepthImagePublisher(img_topic_name)
    rospy.loginfo("Publish {} image to: {}".format(
        args.format, img_topic_name))
    loop_rate = rospy.Rate(args.publish_rate)

    num_total_imgs = len(images_filenames)
    cnt_imgs = 0
    ith_img = 0

    while not rospy.is_shutdown() and cnt_imgs < num_total_imgs:

        # -- Read next image.
        if ith_img == num_total_imgs:
            ith_img = 0
        image = cv2.imread(images_filenames[ith_img], cv2.IMREAD_UNCHANGED)
        cnt_imgs += 1
        ith_img += 1

        # -- Publish data.
        rospy.loginfo("=================================================")
        rospy.loginfo("Publish {}/{}th data; {} published in total.".format(
            ith_img, num_total_imgs, cnt_imgs))

        img_publisher.publish(image, FRAME_ID)
        loop_rate.sleep()
    rospy.logwarn("All images published. Node `{}` stops.".format(node_name))


if __name__ == '__main__':
    node_name = "publish_images"
    if APPEND_RANDOM_NUMBER_TO_NODE_NAME:
        node_name += "_" + str(np.random.randint(low=0, high=99999999999))
    rospy.init_node(node_name)
    args = parse_command_line_arguments()
    main(args)
    rospy.logwarn("Node `{}` stops.".format(node_name))
