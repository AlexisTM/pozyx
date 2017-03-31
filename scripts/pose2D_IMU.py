#!/usr/bin/env python
"""

pose2D_IMU.py: 
    Reads the 2D position from a pozyx tag, and publish the IMU
    to be further processed by imu_tools, http://wiki.ros.org/imu_filter_madgwick.

Author : Alexis PAQUES (@AlexisTM)
"""

import pypozyx
import rospy
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Int16 
from sensor_msgs.msg import MagneticField, Imu
from threading import Thread, Lock

def height_cb(msg):
    global height
    height = msg.data

def imu_publisher():
    global pozyx_sensors
    global ros_mag, ros_imu, local_device, lock
    global remote, serial_port, imuhz

    degreeToRadians = 0.0174533 # radians = degree * degreeToRadians
    mgsTomss = 9.8196/1000.0 # m/s/s = mgs * mgsTomss

    mag_pub = rospy.Publisher('pozyx/mag', MagneticField, queue_size=1)
    imu_pub = rospy.Publisher('pozyx/imu_raw', Imu, queue_size=1)
    
    while not rospy.is_shutdown():
        # doPositioning(pozyx_coords, dimensions, height, algorithm, remote)
        # Coord where to store data
        # Dimensions, 2D/3D
        # Height
        # Algorithm
        # Remote (=Null for self)
        
        lock.acquire()
        local_device.getAllSensorData(pozyx_sensors, remote)
        lock.release()

        now_stamp = rospy.Time.now()
        ros_mag.header.stamp = now_stamp
        ros_imu.header.stamp = now_stamp
        # Pozyx comes in uT, ros in T
        ros_mag.magnetic_field = Vector3(float(pozyx_sensors.magnetic.x)/1000000.0, float(pozyx_sensors.magnetic.y)/1000000.0, float(pozyx_sensors.magnetic.z)/1000000.0)

        # Pozyx comes in dps, ros in rps 
        ros_imu.angular_velocity = Vector3(float(pozyx_sensors.angular_vel.x)*degreeToRadians, float(pozyx_sensors.angular_vel.y)*degreeToRadians, float(pozyx_sensors.angular_vel.z)*degreeToRadians)
        # Pozyx comes in mg/s, ros in  m/s^2 
        ros_imu.linear_acceleration = Vector3(float(pozyx_sensors.linear_acceleration.x)*mgsTomss, float(pozyx_sensors.linear_acceleration.y)*mgsTomss, float(pozyx_sensors.linear_acceleration.z)*mgsTomss)

        mag_pub.publish(ros_mag)
        imu_pub.publish(ros_imu)
        imuhz.sleep()


def coordinate_publisher():
    global pozyx_coords, pozyx_coords_err
    global ros_pose, local_device, lock
    global height, algorithm, remote, serial_port, posehz

    pose_pub = rospy.Publisher('pozyx/pose', PoseWithCovarianceStamped, queue_size=1)
    while not rospy.is_shutdown():
        # doPositioning(pozyx_coords, dimensions, height, algorithm, remote)
        # Coord where to store data
        # Dimensions, 2D/3D
        # Height
        # Algorithm
        # Remote (=Null for self)

        lock.acquire()
        local_device.doPositioning(pozyx_coords, pypozyx.POZYX_2D, height, algorithm, remote_id=remote)
        local_device.getPositionError(pozyx_coords_err, remote)
        lock.release()

        now_stamp = rospy.Time.now()
        ros_pose.header.stamp = now_stamp
        # Pozyx comes in mm, ros in m
        ros_pose.pose.pose.position = Point(float(pozyx_coords.x)/1000.0, float(pozyx_coords.y)/1000.0, float(height)/1000.0)
        ros_pose.pose.covariance = [ float(pozyx_coords_err.x)/1000.0 , float(pozyx_coords_err.xy)/1000.0, float(pozyx_coords_err.xz)/1000.0, 0,0,0,
                                     float(pozyx_coords_err.xy)/1000.0, float(pozyx_coords_err.y)/1000.0 , float(pozyx_coords_err.yz)/1000.0, 0,0,0,
                                     float(pozyx_coords_err.xz)/1000.0, float(pozyx_coords_err.yz)/1000.0, float(pozyx_coords_err.z)/1000.0 , 0,0,0,
                                     0,0,0                                                        , 0,0,0,
                                     0,0,0                                                        , 0,0,0,
                                     0,0,0                                                        , 0,0,0 ]

        pose_pub.publish(ros_pose)
        posehz.sleep()

def init():
    global pozyx_coords, pozyx_coords_err, pozyx_sensors
    global ros_pose, ros_mag, ros_imu, local_device, lock
    global height, algorithm, remote, serial_port, posehz, imuhz

    rospy.init_node('pozyx_pose_node')
    rospy.Subscriber("pozyx/height", Int16, height_cb)

    serial_port = rospy.get_param("port", pypozyx.get_serial_ports()[0].device)
    local_device = pypozyx.PozyxSerial(serial_port)

    # 0 is raw, 1 is fused but not yet implemented, 2 is filtered
    algorithm =  rospy.get_param("algorithm", 2) 
    remote =  rospy.get_param("remote", None)
    height =  rospy.get_param("height", 100)
    posehz =  rospy.Rate(rospy.get_param("posehz", 2))
    imuhz =  rospy.Rate(rospy.get_param("imuhz", 100))

    ros_pose = PoseWithCovarianceStamped()
    ros_mag = MagneticField()
    ros_imu = Imu()
    ros_pose.header.frame_id = "pozyx"
    ros_mag.header.frame_id = "pozyx"
    ros_imu.header.frame_id = "pozyx"

    pozyx_coords = pypozyx.Coordinates()
    pozyx_coords_err = pypozyx.PositionError()
    pozyx_sensors = pypozyx.SensorData()
    
    imu_thread = Thread(target=imu_publisher)
    coord_thread = Thread(target=coordinate_publisher)

    lock = Lock()

    imu_thread.start()
    coord_thread.start()

    rospy.spin()
    
if __name__ == '__main__':
    try:
        init()
        #publisher()
    except rospy.ROSInterruptException:
        pass
        
