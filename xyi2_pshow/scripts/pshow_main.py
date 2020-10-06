#!/usr/bin/env python
# ros support.
import rospy

# matplotlib support.
import numpy as np
import matplotlib.pyplot as plt

# math support.
import math

# topics support.
import std_msgs.msg

# #####################################################################
if __name__ == '__main__':
    rospy.init_node('pshow_main', anonymous=True)
    rate_delay = rospy.Rate(10) # 100hz

    rospy.loginfo(" ")
    rospy.loginfo( "*********************************************")
    rospy.loginfo( "[xyi2_pshow_main]: Showing the beautiful node")
    rospy.loginfo( "*********************************************")
 
    X = np.linspace(-np.pi, np.pi, 256, endpoint=True)
    C,S = np.cos(X), np.sin(X)	
    plt.plot(X,C)
    plt.plot(X,S)

    plt.show()

    try:
	# int part.

    	while not rospy.is_shutdown():
	       

            rate_delay.sleep()

    except rospy.ROSInterruptException:
	
        pass

    #rospy.spin()


