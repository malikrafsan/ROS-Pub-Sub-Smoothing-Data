#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
 
import numpy as np
import math

def sensor_driver_node():
    pub = rospy.Publisher('sensor_measurement', Float64, queue_size=25)
    rospy.init_node('sensor_driver_node')
    rate = rospy.Rate(0.2)

    mu, sigma = 0.0, 0.05
    i = 0
    while not rospy.is_shutdown():
        g = np.random.normal(mu, sigma)
        z_i = math.sin(i * math.pi / 180) + g
        rospy.loginfo("Sensor measurement: " + str(z_i))
        pub.publish(z_i)
        rate.sleep()
        i += 1

if __name__ == '__main__':
    try:
        sensor_driver_node()
    except rospy.ROSInterruptException:
        pass