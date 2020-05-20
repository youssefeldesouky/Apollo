#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range

rospy.init_node("range_sim")
pub = rospy.Publisher("range_sim", Range, queue_size=10)
rate = rospy.Rate(50)

r = Range()
r.header.frame_id = "rear_ultrasonic"
r.radiation_type = r.ULTRASOUND
r.min_range = 0.02
r.max_range = 4.0
r.field_of_view = 0.261799
r.range = 2

while not rospy.is_shutdown():
    try:
        r.header.stamp = rospy.get_rostime()
        pub.publish(r)
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
