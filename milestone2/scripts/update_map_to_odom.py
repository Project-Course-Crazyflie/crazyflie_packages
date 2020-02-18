import math
import numpy as np
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TransformStamped, Vector3
from crazyflie_driver.msg import Position
from Marker import MarkerArray


class MapOdomUpdate:
    def __init__(self):
        # TODO: import MarkerArray
        self.aruco_detect_sub = rospy.Subscriber('/aruco/markers', MarkerArray, self.update_callback)
        self.old_msg = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn = tf2_ros.TransformListener(self.tf_buf)

        self.broadcaster = tf2_ros.TransformBroadcaster()
       
        t = TransformStamped()
        t.frame_id = "map"
        t.child_frame_id = "cf1/odom"
        self.tf_buf.broadcast(t)

    def update_callback(self, m_array):
        if m_array == self.old_msg:
            # Message old 
            return
        self.old_msg = m_array
        
        # TODO: Make possible to update multiple markes?
        m = m_array.markers[0]
        frame_detected = "aruco/detected" + str(m.id)
        frame_map = "aruco/marker" + str(m.id)
        
        if not self.tf_buf.can_transform(frame_detected, frame_map, rospy.Time(0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % frame_map)
            return
        
        transform = self.tf_buf.lookupTransform(frame_map, frame_detected, rospy.Time(0))
        # TODO: outlier detection
        transform.frame_id = "map"
        transform.child_frame_id = "cf1/odom"
        transform.stamp = rospy.Time(0)
        self.tf_buf.broadcast(transform)
        
        
if __name__ == '__main__':
    rospy.init_node('map_to_odom')
    p = MapOdomUpdate()