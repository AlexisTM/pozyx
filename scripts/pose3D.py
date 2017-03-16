#!/usr/bin/env python
"""

pose3D.py : 
    Reads the Pozyx position in 3D and quaternion, then forward it 
    to the ros topic 'pozyx/pose' as a 'geometry_msgs/PoseStamped' message.
    There is no filtering.
    There are outliers when no enough anchors are seen.
    The 3D position is less robust than the 2D one.
    You can also add a laser (Terraranger or LidarLite (LidarEnhanced library)) 
    and update the height of the 2D positioning algorithm.
    
Author : Alexis PAQUES (@AlexisTM)
"""

import pypozyx
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
import rospy


def position_publisher():
    global msg, coords, quat, pose_pub
    try:
        pose_pub = rospy.Publisher('pozyx/pose', PoseStamped, queue_size=1)
        serial_port = rospy.get_param("port", pypozyx.get_serial_ports()[0].device)
        local_device = pypozyx.PozyxSerial(serial_port)
        algorithm =  rospy.get_param("algorithm",0);
        remote = rospy.get_param("remote", None)
    except:
        rospy.loginfo("Serial device Pozyx not found")
        return
    while not rospy.is_shutdown():
        local_device.doPositioning(coords, pypozyx.POZYX_3D, remote_id=remote)
        local_device.getQuaternion(quat)
        msg.pose.position = Point(float(coords.x)/1000.0, float(coords.y)/1000.0, float(coords.z)/1000.0)
        msg.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        msg.header.stamp = rospy.Time.now();
        pose_pub.publish(msg)

def init():
    global msg, coords, quat, pose_pub
    msg = PoseStamped();
    coords = pypozyx.Coordinates()
    quat = pypozyx.Quaternion()
    frame_id =  rospy.get_param("frame_id", "map");
    msg.header.frame_id = frame_id
    rospy.init_node('pozyx_pose_node')

if __name__ == '__main__':
    try:
        init()
        position_publisher()
    except rospy.ROSInterruptException:
        pass
        