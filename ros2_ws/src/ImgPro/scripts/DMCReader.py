#!/usr/bin/env python3
#!/usr/bin/env python3

# ROS client library python imports
import rclpy
from rclpy.node import Node

# ROS messages imports
from sensor_msgs.msg import Image as ROSImageMsg

# General imports
import cv2  # OpenCV
from cv_bridge import CvBridge  # OpenCV bridge for image conversion to ROS messages

import time
import threading
import subprocess
from matplotlib import pyplot as plt

#DMC decoding imports
from pylibdmtx.pylibdmtx import decode

class DMCReader(Node):

    # Private member variables
    


    def __init__(self,
                 name: str = "DMCReader_node",
                 topic: str = "camera_stream2D",
                 ) -> None:
        """
        Class constructor
        :param name: The name of the node, default: DMCReader_node
        :param topic: The topic for the 2D camera stream of the node, default: camera_stream2D
        """

        # Setting up name of the node 
        super().__init__(name)

        # Setting up the subscriber
        self.video_subscriber_ = self.create_subscription(
            ROSImageMsg,
            topic,
            self.DMCReader_,
            10
        )

        # Create a cv bridge instance
        self.cv_br_ = CvBridge()


    def DMCReader_(self, msg_data) -> None:
        """
        Subscriber callback
        """

        # Get the message information
        time_stamp = msg_data.header.stamp
        frame_id = msg_data.header.frame_id

        # Convert message to image
        image = self.cv_br_.imgmsg_to_cv2(msg_data)

        
        
        
        """
         Detects the DMCs in the image
        :max_count: To specify how many number of DMCs to be detcted 
        :timeout  : Maximum number of seconds it could take
        
        """

           
        codes = decode(image, max_count=5, timeout=5000)
        
        # Initlialize the list to store detected DMCs
        dmc_decoded = []
        
        # Every DMC decoded is appended to the list and and prints the result after decoding all of the DMCs
        for code in codes:
            dmc_decoded.append(code.data.decode())
        print(dmc_decoded)
        

        
        if dmc_decoded:
            print("DMCs found")
            image_decoded = image

            image_height =image.shape[0]

            for idx, dmc in zip(range(len(codes)), codes):
                # print("{idx}: {dmc}".format(idx, dmc))

                (x, y, w, h) = dmc.rect
                y = int(image_height / 2 - (y - image_height / 2))
                image_decoded = cv2.rectangle(image_decoded, (x, y), (x + w, y - h), (255, 0, 0), 10)

                origin = (int(x + w / 2), int(y + h / 2))
                dmc_content = dmc.data.decode('ascii')
                offset = (int(-len(dmc_content) * 5 * 2.5), int(1.25 * h))
                position = tuple([sum(e) for e in zip(origin, offset)])

                cv2.putText(
                    image_decoded,
                    dmc_content,
                    position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    2.5,
                    (255, 0, 0),
                    5
                )

            plt.subplot(1, 2, 1)
            plt.imshow(image)
            plt.title("Snapshot")

            plt.subplot(1, 2, 2)
            plt.imshow(image_decoded)
            plt.title("Decoded")
            plt.show()                                                                      



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
    # Create a instance of the DMC reader
    
    dmcreadernode = DMCReader(
        #node_name,
        #node_topic
    )

    # Let the instance be executed continuously
    rclpy.spin(dmcreadernode)

    # Collect the garbage if the dmcreadernode is finished
    dmcreadernode.destroy_node()

    # Shut the ROS client library down
    rclpy.shutdown()


# Main clause, run main if this is the top level script
if __name__ == '__main__':
    main()

