#!/usr/bin/env python3
#!/usr/bin/env python3

# ROS client library python imports
import rclpy
from rclpy.node import Node

# ROS messages imports
from sensor_msgs.msg import Image as ROSImageMsg

# General imports
import cv2 as cv # OpenCV
from cv_bridge import CvBridge  # OpenCV bridge for image conversion to ROS messages


from matplotlib import pyplot as plt
import numpy as np
import math


class Homography(Node):

    # Private member variables
    


    def __init__(self,
                 name: str = "ImageRegistration_node",
                 topic: str = "camera_stream2D",
                 ) -> None:
        """
        Class constructor
        :param name: The name of the node, default: ImageRegistration_node
        :param topic: The topic for the 2D camera stream of the node, default: camera_stream2D
        """

        # Setting up name of the node
        super().__init__(name)

        # Setting up the subscriber
        self.video_subscriber_ = self.create_subscription(
            ROSImageMsg,
            topic,
            self.Homography_,
            10
        )

        # Create a cv bridge instance
        self.cv_br_ = CvBridge()


    def Homography_(self, msg_data) -> None:
        """
        Subscriber callback
        """

        # Get the message information
        time_stamp = msg_data.header.stamp
        frame_id = msg_data.header.frame_id

        # Convert message to image
        image = self.cv_br_.imgmsg_to_cv2(msg_data)
    

        img1 = cv.imread('/home/developer/ws/src/ImgPro/Feature/Reference.jpg', 0)  # referenceImage
        img2 = cv.cvtColor(image,0)  # sensedImage
        
        
        scale_percent = 20 # percent of original size
        width1 = int(img1.shape[1] * scale_percent / 100)
        height1 = int(img1.shape[0] * scale_percent / 100)
        dim1 = (width1, height1)
    
              
        scale_percent1 = 20
        width2 = int(img2.shape[1] * scale_percent1 / 100)
        height2 = int(img2.shape[0] * scale_percent1 / 100)
        dim2 = (width2, height2)
        

        # resize image
        img1 = cv.resize(img1, dim1, interpolation=cv.INTER_AREA)
        img2 = cv.resize(img2, dim1, interpolation=cv.INTER_AREA)

        # Initiate ORB detector
        orb = cv.ORB_create(2000)

        # find the keypoints and descriptors with ORB
        kp1, des1 = orb.detectAndCompute(img1,None)
        kp2, des2 = orb.detectAndCompute(img2,None)


        # create BruteForce matcher object
        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        # Match descriptors.
        matches = bf.match(des1,des2)

        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)

        good_matches = matches[:600]
        for i, m in enumerate(matches):
            if i < len(matches) - 1 and m.distance < 0.7 * matches[i+1].distance:
                good_matches.append(m)

        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches     ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)

        # Find the homography transformation between source and target points
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        #h,w = img1.shape[:2]
        #pts = np.float32([ [0,0],[0,height2-1],[width2-1,height2-1],[width2-1,0] ]).reshape(-1,1,2)
        h, w = img1.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)


        dst = cv.perspectiveTransform(pts,M)
        dst += (w, 0)  # adding offset

        #Get the bounding box points
        bbpt = np.array(dst)

        #Define origin coordinates for reference
        x0= np.int32(bbpt[1, 0, 0])
        y0= np.int32(bbpt[1, 0, 1])
        origin= (x0,y0)
        origin=(0,0)
        #Get the (x0, x1) of the bounding box for grip point
        x01 = np.int32(bbpt[0, 0, 0])

        #Get the (y0, y1) of the bounding box for grip point
        y01= np.int32(bbpt[0, 0, 1])

        """
        Calculate the Grip point2 coordinates
        Mid_pointx01: Mid point of x0 and x1
        Mid_pointy01: Mid point of y0 and y1
        """

        Mid_pointx01 = np.int32(((x0+x01)/2))
        Mid_pointy01 = np.int32(((y0+y01)/2))

        # Calculate the Grip point1 coordinates
        Grip_point1=(Mid_pointx01, Mid_pointy01)

        # Calculate the x3 cordinates
        x3= np.int32(bbpt[3, 0, 0])
        y3= np.int32(bbpt[3, 0, 1])


        #Get the (x3, x2) of the bounding box for grip point
        x32 = np.int32(bbpt[2, 0, 0])

        #Get the (y3, y2) of the bounding box for grip point
        y32= np.int32(bbpt[2, 0, 1])

        """
        Calculate the Grip point2 coordinates
        Mid_pointx32: Mid point of x3 and x2
        Mid_pointy32: Mid point of y3 and y2
        """
        Mid_pointx32 = np.int32(((x3+x32)/2))
        Mid_pointy32 = np.int32(((y3+y32)/2))

        # Calculate the Grip point2 coordinates
        Grip_point2=(Mid_pointx32, Mid_pointy32)


        draw_params = dict(matchColor = (20,20,20), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = matchesMask, # draw only inliers
                    flags = 2)

        img3 = cv.drawMatches(img1,kp1,img2,kp2,good_matches, None,**draw_params)

        # Draw bounding box in Red
        img3 = cv.polylines(img3, [np.int32(dst)], True, (255,255,255),3, cv.LINE_AA)
         
        # Mark the bottom-left corner of the bounding box as origin
        cv.circle(img3, origin, 5, (110, 127, 5), -1)
        cv.putText(img3, "Origin", (x0+10, y0+10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        #Mark the first Grip point
        cv.circle(img3, Grip_point1, 5, (110, 127, 5), -1)
        cv.putText(img3, "Grip_point1", (Mid_pointx01+10, Mid_pointy01+10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        #Mark the second Grip point
        cv.circle(img3, Grip_point2, 5, (110, 127, 5), -1)
        cv.putText(img3, "Grip_point2", (Mid_pointx32+10, Mid_pointy32+10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        img3 = cv.resize(img3, dim1, interpolation=cv.INTER_AREA)   
        #Show the resultant image
        cv.imshow("result", img3)
        print(np.int32(dst))


        print("origin is ",origin)
        print("Grip_point1 is ",Grip_point1)
        print("Grip_point2 is", Grip_point2)

        cv.waitKey()                                                                   


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
    
    Homographynode = Homography(
        #node_name,
        #node_topic
    )

    # Let the instance be executed continuously
    rclpy.spin(Homographynode)

    # Collect the garbage if the node is finished
    Homographynode.destroy_node()

    # Shut the ROS client library down
    rclpy.shutdown()


# Main clause, run main if this is the top level script
if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()


