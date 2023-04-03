#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Vector3
import geometry_msgs.msg
import tf2_ros
import tf_conversions

class CoordToPosition:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,queue_size=1)
        self.br = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher("object_pos_to_base_link", Vector3, queue_size=1)
        self.sub = rospy.Subscriber("/object_coordinate", Vector3, self.transform_camera_link_to_base_link)
        self.first = True

    def transform_camera_link_to_base_link(self, data):
        t = geometry_msgs.msg.TransformStamped()
        base = geometry_msgs.msg.TransformStamped()

        base.header.stamp = rospy.Time.now()
        base.header.frame_id = "base_link"
        base.child_frame_id = "camera_base"
        base.transform.translation.x = -0.0611
        base.transform.translation.y = -0.0611
        base.transform.translation.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0.25, -2.1817) # orginially 2.35619 which 135 degrees
        base.transform.rotation.x = q[0]
        base.transform.rotation.y = q[1]
        base.transform.rotation.z = q[2]
        base.transform.rotation.w = q[3]

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_base"
        t.child_frame_id = "object"
        t.transform.translation.x = data.z/1000
        t.transform.translation.y = -data.x/1000
        t.transform.translation.z = -data.y/1000

        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(base)
        self.br.sendTransform(t)
        if(self.first == True):
            self.first = False
        else:
            trans = self.tfBuffer.lookup_transform('base_link', 'object', rospy.Time())
            result_msg = Vector3()
            result_msg.x = trans.transform.translation.x
            result_msg.y = trans.transform.translation.y
            result_msg.z = trans.transform.translation.z
            self.pub.publish(result_msg)

def main():
    listener = CoordToPosition()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('tf_to_position', anonymous=True)
    main()
        