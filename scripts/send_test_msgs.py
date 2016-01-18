#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_plugin_tutorials' )
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
import rospy
from math import cos, sin
import tf

topic = 'test_point'
publisher = rospy.Publisher( topic, PointStamped )

rospy.init_node( 'test_point' )

br = tf.TransformBroadcaster()
rate = rospy.Rate(100)
radius = 2
angle = 0

dist = 3
while not rospy.is_shutdown():

    point = PointStamped()
    point.header.frame_id = "/base_link"
    point.header.stamp = rospy.Time.now()
   
    point.point.x = 2 * sin( 10 * angle )
    point.point.y = sin( 20 * angle )
    point.point.z = 0.1

    publisher.publish( point )

    br.sendTransform((2 * cos(2*angle), 1 * sin(angle), 0),
                     tf.transformations.quaternion_from_euler(0, 0, angle),
                     rospy.Time.now(),
                     "base_link",
                     "map")
    angle += .01
    rate.sleep()

