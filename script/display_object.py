#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Vector3
import geometry_msgs.msg
import tf2_ros
import tf_conversions
import math

class CoordToTF:
    def __init__(self):
        self.sub = rospy.Subscriber("/object_coordinate", Vector3, self.publish_tf)
        

    def publish_tf(self, data):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        base = geometry_msgs.msg.TransformStamped()

        base.header.stamp = rospy.Time.now()
        base.header.frame_id = "base_link"
        base.child_frame_id = "arm_base"
        base.transform.translation.x = 0
        base.transform.translation.y = 0
        base.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0.3927, -2.35619)
        base.transform.rotation.x = q[0]
        base.transform.rotation.y = q[1]
        base.transform.rotation.z = q[2]
        base.transform.rotation.w = q[3]

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "arm_base"
        t.child_frame_id = "object"
        t.transform.translation.x = data.z/1000
        t.transform.translation.y = -data.x/1000
        t.transform.translation.z = -data.y/1000

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(base)
        br.sendTransform(t)


def main():
    listener = CoordToTF()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('coord_to_tf', anonymous=True)
    main()

