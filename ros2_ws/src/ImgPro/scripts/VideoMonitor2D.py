#!/usr/bin/env python3

"""
------------------------------------------------------------------------------
VideoMonitor2D.py
------------------------------------------------------------------------------
Author: Vinayaka Bhat
Package: ImgPro
Node Name: VideoMonitor2D_node
Date: 07/05/2024
Description: Receives a 2D image stream (static or live) as a ROS2 
             sensor_msgs/Image message for usage/processing.
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


class VideoMonitor2D(Node):

    # Private member variables
    topic_ = None
    text_font_ = None
    text_scale_ = None
    text_color_ = None
    text_thickness_ = None

    def __init__(self,
                 name: str = "VideoMonitor2D_node",
                 topic: str = "camera_stream2D",
                 ) -> None:
        """
        Class constructor
        :param name: The name of the node, default: VideoPublisher2D_node
        :param topic: The topic for the 2D camera stream of the node, default: camera_stream2D
        """

        # Setting up name of the node
        super().__init__(name)

        # Setting up the subscriber
        self.video_subscriber_ = self.create_subscription(
            ROSImageMsg,
            topic,
            self.monitor_,
            10
        )

        # Create a cv bridge instance
        self.cv_br_ = CvBridge()

        # Set up printing information
        self.topic_ = topic
        self.text_font_ = cv2.FONT_HERSHEY_SIMPLEX
        self.text_scale_ = 1
        self.text_color_ = (0, 0, 0)
        self.text_thickness_ = 2

    def monitor_(self, msg_data) -> None:
        """
        Subscriber callback
        """
        self.get_logger().info('Receive message')
        # Get the message information
        time_stamp = msg_data.header.stamp
        frame_id = msg_data.header.frame_id

        # Convert message to image
        image = self.cv_br_.imgmsg_to_cv2(msg_data)

        # Determine the text size
        """
        textsize_stamp = cv2.getTextSize(
            time_stamp,
            self.text_font_,
            self.text_scale_,
            self.text_thickness_
        )
        textsize_frame_id = cv2.getTextSize(
            frame_id,
            self.text_font_,
            self.text_scale_,
            self.text_thickness_,
            0
        )

        # Calculate the top left corner
        pos_X_stamp = 0 + 15 + textsize_stamp.width
        pos_Y_stamp = 0 + 15 + textsize_stamp.height
        pos_X_frame_id = pos_X_stamp
        pos_Y_frame_id = pos_Y_stamp + textsize_stamp.height + self.text_thickness_

        # Insert the time stamp
        image = cv2.putText(
            image,
            time_stamp,
            (pos_X_stamp, pos_Y_stamp),
            self.text_font_,
            self.text_scale_,
            self.text_color_,
            self.text_thickness_,
            cv2.FILLED
        )

        # Insert the frame id
        image = cv2.putText(
            image,
            time_stamp,
            (pos_X_frame_id, pos_Y_frame_id),
            self.text_font_,
            self.text_scale_,
            self.text_color_,
            self.text_thickness_,
            cv2.FILLED
        )
    """
        # Display image
        cv2.imshow(self.topic_, image)
        cv2.waitKey(1)


def main(args=None) -> None:
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Check for attributes
    """
    if args:
	    if "name" in args:
		node_name = args["name"]
	    if "topic" in args:
		node_topic = args["topic"]
    """
    # Create a instance of the monitor
    
    video_monitor = VideoMonitor2D(
        #node_name,
        #node_topic
    )

    # Let the instance be executed continuously
    rclpy.spin(video_monitor)

    # Collect the garbage if the node is finished
    video_monitor.destroy_node()

    # Shut the ROS client library down
    rclpy.shutdown()


# Main clause, run main if this is the top level script
if __name__ == '__main__':
    main()

