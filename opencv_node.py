#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import UInt16MultiArray    
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys

bridge = CvBridge()
centroid_pub = rospy.Publisher("centroids", UInt16MultiArray)

def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show:
        cv2.imshow("RGB Image", rgb_image)
    return rgb_image

def filter_colour(rgb_image, lower_bound_colour, upper_bound_colour):
    #convert the image into the HSV colour space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_image, lower_bound_colour, upper_bound_colour)

    return mask

def image_callback(ros_image):
    global bridge
    global centroid_pub
    #convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        depth_array = np.array(cv_image, dtype=np.float32)
        print(depth_array.shape)
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv
    clothUpper = (27, 255, 255)
    clothLower = (0, 126, 128)
    binaryImage = filter_colour(cv_image, clothLower, clothUpper)

    bgrImage = cv2.cvtColor(binaryImage, cv2.COLOR_GRAY2BGR)

    # Apply an erosion + dilation to get rid of small noise:

    # Set kernel (structuring element) size:
    kernelSize = 3

    # Set operation iterations:
    opIterations = 3

    # Get the structuring element:
    maxKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernelSize, kernelSize))

    # Perform closing:
    openingImage = cv2.morphologyEx(binaryImage, cv2.MORPH_OPEN, maxKernel, None, None, opIterations, cv2.BORDER_REFLECT101)

    contours, _ = cv2.findContours(openingImage, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
    centres = []
    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])         
        if area < 1000:
            continue  
        imageMoments = cv2.moments(contours[i])
                # Compute centroid
        if imageMoments["m00"] != 0:
            cx = int(imageMoments['m10']/imageMoments['m00'])
            cy = int(imageMoments['m01']/imageMoments['m00'])
        else:
            # set values as what you need in the situation
            cx, cy = 0, 0

        centres.append([cx, cy])
        cv2.circle(bgrImage, tuple(centres[-1]), 3, (0, 255, 0), -1)

    print(centres)
    centre_msg = UInt16MultiArray()
    centre_msg.data = centres
    centroid_pub.publish(centre_msg)
    cv2.imshow("centroid", bgrImage)
    cv2.waitKey(3)

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
  image_sub = rospy.Subscriber("/camera/color/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)