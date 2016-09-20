#!/usr/bin/env python

import rospy
import roslaunch
from math import *
from time import sleep
from arduino_mr.msg import arduino_input, feedback_int
from geometry_msgs.msg import Vector3, Vector3Stamped

package = 'rosserial_python'
executable = 'serial_node.py'
serial_port = '/dev/ttyACM0'
node = roslaunch.core.Node(package, executable, args=serial_port)

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print process.is_alive()

class ATX2():
	def __init__(self):
		self.r = 200
		self.pub = rospy.Publisher('arduino_input_2at', arduino_input)#, queue_size=3)	
		self.pub_odom = rospy.Publisher('/pioneer/odom', Vector3Stamped)#, queue_size=1)	
		rospy.Subscriber("/feedback_2at", feedback_int, self.updateMROdometry)	


	def shutdown(self):
		for i in range (0,100):
			self.pub.publish(mode=3, data1=0, data2=0)
			self.r.sleep()
		print 'Safe shutdown'
		process.stop()
		
        def updateMROdometry(self, msg):
    
            R = 0.22/2.0
            d = 0.1905*1.63
            b = 0.1905
            nnpulley = 79.66215
            encres = 8187.5
            looptime = 15.0 * 10.0**(-3.0)
    
            encoderR =  msg.encoderR
            encoderL =  msg.encoderL

            wL = 2.0*pi*(encoderL/encres)/0.015
            wR = 2.0*pi*(encoderR/encres)/0.015
            dtheta = ((encoderR-encoderL)*2.0*pi*R)/(2.0*encres*d)
            omega = -((wR-wL)*R)/(d*2.0)
            u = (wR+wL)*R/2.0
    
            pub_msg = Vector3Stamped()
            pub_msg.header.stamp =  rospy.Time.now()
            pub_msg.vector.x = u
            pub_msg.vector.y = omega
            pub_msg.vector.z = 0.0
            
            self.pub_odom.publish(pub_msg)

if __name__ == '__main__':
        rospy.init_node('pioneer_atx_node')
        obj = ATX2()
        rate_it = rospy.Rate(obj.r)
	try:
	    while not rospy.is_shutdown():
	        rate_it.sleep()    
		#safe_shutdown()
            rospy.spin()
	except rospy.ROSInterruptException:
		pass

		
