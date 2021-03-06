#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3

def transform_from_marker(m):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'aruco/marker' + str(m['id'])
    t.transform.translation = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t

def transform_from_sign(m):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id =  'sign/' + m['sign']
    t.transform.translation = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t


def main(argv=sys.argv):
    # Let ROS filter through the arguments
    #args = rospy.myargv(argv=argv)
    rospy.init_node('displaymapmarkers')
    # Load world JSON
    w_name = rospy.get_param(rospy.get_name() + "/world_name")
    rospy.loginfo(w_name)


    with open(w_name, 'rb') as f:
        world = json.load(f)

    # Create a transform for each marker
    transforms = [transform_from_marker(m) for m in world['markers']]
    transforms_signs = [transform_from_sign(m) for m in world['roadsigns']]


    # Publish these transforms statically forever
    transforms = transforms + transforms_signs
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.spin()

if __name__ == "__main__":
    main()
