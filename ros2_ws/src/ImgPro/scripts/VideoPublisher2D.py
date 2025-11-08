#!/usr/bin/env python3
"""
------------------------------------------------------------------------------
VideoPublisher2D.py
------------------------------------------------------------------------------
Author: Vinayaka Bhat
Package: ImgPro
Node Name: VideoPublisher2D_node
Date: 07/05/2024
Description: Publishes a 2D image stream (static or live) as a ROS2 
             sensor_msgs/Image message for downstream processing.
------------------------------------------------------------------------------
"""


# ROS client library python imports
import rclpy
from rclpy.node import Node

# ROS messages imports
from sensor_msgs.msg import Image as ROSImageMsg

# General imports
import cv2  # OpenCV
from cv_bridge import CvBridge  # OpenCV bridge for image conversion to ROS messages


class VideoPublisher2D(Node):

    # Private member variable
    frame_id_ = None

    def __init__(self,
                 name: str = "VideoPublisher2D_node",
                 topic: str = "camera_stream2D",
                 frame_id: str = "2DCamera",
                 frequency: float = 20.0,
                 device_name: int = 2
                 ) -> None:
        """
        Class constructor
        :param name: The name of the node, default: VideoPublisher2D_node
        :param topic: The topic for the 2D camera stream of the node, default: camera_stream2D
        :param frame_id: The frame/link with which the camera stream is associated, default: 2DCamera 
        :param frequency: The publishing frequency of the camera stream, default: 20.0 [Hz]
        :param device_name: The device the video stream should be generated from, default: 0
        """

        # Setting up name of the node
        super().__init__(name)

        # Setting up the publisher
        self.video_publisher_ = self.create_publisher(
            ROSImageMsg,
            topic,
            10
        )

        # Setting up the timer
        self.video_publish_timer_ = self.create_timer(
            1/frequency,
            self.publish_
        )

        # Create a OpenCV image laoding
        self.cap_ = cv2.imread('/home/developer/ROS_Foxy_ModProFT/src/ImgPro/Feature/source.jpg', 0)

        

        # Create a cv bridge instance
        self.cv_br_ = CvBridge()

        self.frame_id_ = frame_id

    def publish_(self) -> None:
        """
        Timer callback
        """

        # Capture frame
        frame = self.cap_
        

        
        # Create the ROS message
        img_msg = self.cv_br_.cv2_to_imgmsg(frame)
        # Add current timestamp
        img_msg.header.stamp = self.get_clock().now().to_msg()
        # Add the frame_id
        img_msg.header.frame_id = self.frame_id_

        self.video_publisher_.publish(img_msg)


def main(args=None) -> None:
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Check for attributes
    """
    if "name" in args:
        node_name = args["name"]
    if "topic" in args:
        node_topic = args["topic"]
    if "frame_id" in args:
        frame_id = args["frame_id"]
    if "frequency" in args:
        node_frequency = args["frequency"]
    if "device_name" in args:
        device_name = args["device_name"]
    """
    # Create a instance of the publisher
    video_publisher = VideoPublisher2D(
        #node_name,
        #node_topic,
        #frame_id,
        #node_frequency,
        #device_name
    )

    # Let the instance be executed continuously
    rclpy.spin(video_publisher)

    # Collect the garbage if the node is finished
    video_publisher.destroy_node()

    # Shut the ROS client library down
    rclpy.shutdown()


# Main clause, run main if this is the top level script
if __name__ == '__main__':
    main()
