#!/usr/bin/env python
"""

range.py :
    Range between two sensors

Author : Alexis PAQUES (@AlexisTM)
"""

import pypozyx
import rospy
from sensor_msgs.msg import Range

def range_publisher():
    global msg, device_range, range_pub
    try:
        range_pub = rospy.Publisher('pozyx/range', Range, queue_size=1)
        serial_port = rospy.get_param("port", pypozyx.get_serial_ports()[0].device)
        local_device = pypozyx.PozyxSerial(serial_port)
        algorithm =  rospy.get_param("algorithm", 0)
        remote =  rospy.get_param("remote", None) # From where, None is local
        destination =  rospy.get_param("destination", 0x6042) # To where 
    except:
        rospy.loginfo("Serial device Pozyx not found")
        return
    while not rospy.is_shutdown():
        # doRanging : Destination, Device_range, Remote

        local_device.doRanging(destination, device_range, remote)
        msg.range = device_range.distance
        range_pub.publish(msg)

def init():
    global msg, device_range, range_pub
    msg = Range();
    msg.radiation_type = 2
    msg.field_of_view = 3.1415
    msg.min_range = 0.2
    msg.max_range = 10.0 # depends on the configuration, 300m on datasheet, 10m reported by users 
    device_range = pypozyx.DeviceRange()
    rospy.init_node('pozyx_pose_node')

if __name__ == '__main__':
    try:
        init()
        range_publisher()
    except rospy.ROSInterruptException:
        pass
        



