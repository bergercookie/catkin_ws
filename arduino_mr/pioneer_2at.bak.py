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

class safe_shutdown():
	def __init__(self):
		rospy.init_node('safe_shutdown', anonymous=True)
		rospy.on_shutdown(self.shutdown)
		self.r = rospy.Rate(200)
		self.pub = rospy.Publisher('arduino_input_2at', arduino_input)#, queue_size=3)	
		self.pub_odom = rospy.Publisher('/pioneer/odom', Vector3Stamped)#, queue_size=1)	
		rospy.Subscriber("/feedback_2at", feedback_int, self.updateMROdometry)	

		while not rospy.is_shutdown():
			self.r.sleep()

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

            wL = 2.0*math.pi*(encoderL/encres)/0.015
            wR = 2.0*math.pi*(encoderR/encres)/0.015
            dtheta = ((encoderR-encoderL)*2.0*math.pi*R)/(2.0*encres*d)
            omega = -((wR-wL)*R)/(d*2.0)
            u = (wR+wL)*R/2.0
    
            pub_msg = Vector3Stamped()
            pub_msg.header.stamp =  rospy.Time.now()
            pub_msg.x = u
            pub_msg.y = omega
            pub_msg.z = 0.0
            
            self.pub_odom.publish(pub_msg)

if __name__ == '__main__':
        rospy.init_node('pioneer_atx_node')
        obj = safe_shutdown()
	try:
	    while not rospy.is_shutdown():
	              obj.r.sleep() 
		#safe_shutdown()
	except rospy.ROSInterruptException:
		pass

		
