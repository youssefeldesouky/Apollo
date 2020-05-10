#!/usr/bin/env python

import rosbag

import datetime
from tf.msg import tfMessage
from argparse import ArgumentParser
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import MagneticField
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_multiply,quaternion_matrix
import tf

def main():
    bag = rosbag.Bag("imu_record.bag")
    for i in bag.read_messages(topics=("/imu/data",)):
        print(i)

if __name__ == "__main__":
    main()
