#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from object_detection.msg import PixelsAndDepth

import pyrealsense2 as rs2
from geometry_msgs.msg import Vector3

class PixelToCoord:
    def __init__(self):
        self.pub = rospy.Publisher("object_coordinate", Vector3, queue_size=1)
        self.sub = rospy.Subscriber('/pixel_and_depth', PixelsAndDepth, self.convertPixelsToCoordinates)
        self.sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)
        self.intrinsics = None

    def convertPixelsToCoordinates(self,data):
        if self.intrinsics:
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [data.X, data.Y], data.Z)
            result_msg = Vector3()
            result_msg.x = result[0]
            result_msg.y = result[1]
            result_msg.z = result[2]
            self.pub.publish(result_msg)



    def imageDepthInfoCallback(self, cameraInfo):
        # import pdb; pdb.set_trace()
        if self.intrinsics:
            return
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.K[2]
        self.intrinsics.ppy = cameraInfo.K[5]
        self.intrinsics.fx = cameraInfo.K[0]
        self.intrinsics.fy = cameraInfo.K[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            self.intrinsics.model = rs2.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo.D]

def main():
    listener = PixelToCoord()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pixel_to_coord', anonymous=True)
    main()