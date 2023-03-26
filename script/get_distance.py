#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray
import numpy as np

centres = [[]]

def convert_depth_image(ros_image):
    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        depths = []
        if(centres.size != 0):
            for points in centres:
                depths.append(depth_array[points[0],points[1]])
            print(depths)
        centres = [[]]
    except CvBridgeError as e:
        print(e)
     #Convert the depth image to a Numpy array
    
def get_centres(centres_list):
    global centres
    centres = np.array(centres_list)



if __name__ == '__main__':
    rospy.init_node('pixel2depth',anonymous=True)
    depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
    centre_sub = rospy.Subscriber("/centroids", UInt16MultiArray, callback = get_centres, queue_size= 1)
    rospy.spin()
