#!/usr/bin/env python
"""
setup_anchors.py : 
    Reads the pozyx anchors config and tags config from the rosparam service
    Set the network configuration to all

Author : Alexis PAQUES (@AlexisTM)
"""

import pypozyx
import rospy

# adding None will cause the local device to be configured for the anchors as well.

# Configuration in mm !

def set_anchor_configuration():
    global anchors, tags_id

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
        serial_port = rospy.get_param("port", pypozyx.get_serial_ports()[0].device)
        local_device = pypozyx.PozyxSerial(serial_port)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    for tag in tags_id:
        for anchor in anchors:
            pozyx.addDevice(anchor, tag)
        if len(anchors) > 4:
            pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
                                        len(anchors), remote_id=tag)
            pozyx.saveRegisters(settings_registers, remote_id=tag)
        pozyx.saveNetwork(remote_id=tag)
        if tag is None:
            rospy.loginfo("Local device configured")
        else:
            rospy.loginfo("Device with ID 0x%0.4x configured." % tag)
    rospy.loginfo("Configuration completed! Shutting down node now...")

def init(): 
    global anchors, tags_id
    rospy.init_node('pozyx_configure')
    rospy.loginfo("Configuring device list.")
    tags_param = rospy.get_param("tags/addresses", [])
    tags_id = [None] + tags_param
    anchors_param = rospy.get_param("anchors", [])
    anchors = []
    rospy.loginfo(str(anchors_param))
    rospy.loginfo(str(tags_param))
    for anchor in anchors_param:
        rospy.loginfo(str(anchor))
        rospy.loginfo(str(anchor["address"]))
        anchors.append(pypozyx.DeviceCoordinates(anchor["address"], anchor["flag"], pypozyx.Coordinates(anchor["position"][0], anchor["position"][1], anchor["position"][2])))

if __name__ == '__main__':
    try:
        init()
        set_anchor_configuration()
    except rospy.ROSInterruptException:
        pass
