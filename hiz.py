#!/usr/bin/env python
import cv2
import rospy
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import numpy as np
from std_srvs.srv import *
from math import pow,atan2,sqrt
hiz = 0.5
durak = 0.4


def imageCallback(data):
	frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
	rate = rospy.Rate(10)
		
	rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR2RGB
    
	alt_mavi = np.array([60, 40, 40])
	ust_mavi = np.array([150, 255, 255])
	maviMaske = cv2.inRange(rgb, alt_mavi, ust_mavi)
	cv2.imshow("Goruntu",rgb)
	cv2.imshow("mavi",maviMaske)
    	
	cv2.waitKey(3)
		
	rate.sleep()
	
	
def scan_callback(msg):
	FRONT = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
	RIGHT = min(msg.ranges[300:345])
	LEFT = min(msg.ranges[15:60])
	pub = rospy.Publisher('/cmd_vel',Twist, queue_size = 10)
	vel_msg = Twist()
	rate = rospy.Rate(10)
	global durak	
	if FRONT < durak or RIGHT < 0.2:
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0.2
		durak = 0.6
	else:
		vel_msg.linear.x = hiz
		vel_msg.angular.z = 0
		durak = 0.4
		
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	pub.publish(vel_msg)
	
	print("Scanner data FRONT: %.2f RIGHT: %.2f LEFT : %.2f" % (FRONT,RIGHT,LEFT))
    
	rate.sleep()
        
		




if __name__ == '__main__':
	try:
		rospy.init_node('turtlebot_control', anonymous=True)
		restartSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
		restartSimulation()
		rospy.Subscriber('/scan', LaserScan, scan_callback)
		rospy.Subscriber("/camera/image", numpy_msg(Image), imageCallback)
		rospy.spin()
        
       
	except rospy.ROSInterruptException: 
		# clears all window buffers
		pass
