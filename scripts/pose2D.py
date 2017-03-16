#!/usr/bin/env python
"""

pose2D.py: 
    Reads the Pozyx position and quaternion in 2D, then forward it 
    to the ros topic 'pozyx/pose2D' as a 'geometry_msgs/PoseStamped' message.
    Use this node multiple times using the namespace attribute (<node ns="...">)
    The height is set before to let the algorithm locate the tag.
    There is no filtering.
    There are outliers when no enough anchors are seen.
    Should implement the 3D algorithm using a terraranger as height.

Author : Alexis PAQUES (@AlexisTM)
"""

import pypozyx
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped


def position_publisher():
    global msg, coords, quat, pose_pub
    try:
        pose_pub = rospy.Publisher('pozyx/pose2D', PoseStamped, queue_size=1)
        serial_port = rospy.get_param("port", pypozyx.get_serial_ports()[0].device)
        local_device = pypozyx.PozyxSerial(serial_port)
        algorithm =  rospy.get_param("algorithm",0)
        height =  rospy.get_param("height", 100)
        remote =  rospy.get_param("remote", None)
    except:
        rospy.loginfo("Serial device Pozyx not found")
        return
    while not rospy.is_shutdown():
        # doPositioning(coords, dimensions, height, algorithm, remote)
        # Coord where to store data
        # Dimensions, 2D/3D
        # Height
        # Algorithm
        # Remote (=Null for self)
        local_device.doPositioning(coords, pypozyx.POZYX_2D, height, algorithm, remote_id=remote)
        local_device.getQuaternion(quat)
        msg.pose.position = Point(float(coords.x)/1000.0, float(coords.y)/1000.0, float(height)/1000.0)
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
        
