#!/usr/bin/env python

# Import libraries
import rospy
from std_msgs.msg import Float64
import numpy as np
import math

def sensor_driver_node():
    # I.S. any state
    # F.S. Publishes message (z_i) of type float64 to /sensor_measurement topic where 
    # z_i = sin(i * pi / 180) + g, where g is a random number with Gaussian distribution of mean=0.0 and sigma=0.05.

    # Initialize Node, Node's publisher object, and Node loop frequency
    pub = rospy.Publisher('sensor_measurement', Float64, queue_size=25)
    rospy.init_node('sensor_driver_node')
    rate = rospy.Rate(100)

    # Define mean and sigma
    mu, sigma = 0.0, 0.05
    
    # Sending message until Node is shutdown
    i = 0
    while not rospy.is_shutdown():
        # Calculate random number
        g = np.random.normal(mu, sigma)

        # Calculate value of z_i
        z_i = math.sin(i * math.pi / 180) + g

        # Print message on the screen
        rospy.loginfo("Sensor measurement: " + str(z_i))
        
        # Publish the message
        pub.publish(z_i)

        # Sleep until correct timing of looping
        rate.sleep()
        i += 1

# MAIN PROGRAM
if __name__ == '__main__':
    try:
        sensor_driver_node()
    
    # Catch interruption signal
    except rospy.ROSInterruptException:
        pass