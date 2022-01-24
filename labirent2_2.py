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
hiz = 0.4
durak = 0.4
near_wall = 0

def imageCallback(data):
	frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
	rate = rospy.Rate(10)
		
	rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #BGR2RGB
    
	alt_mavi = np.array([60, 40, 40])
	ust_mavi = np.array([150, 255, 255])
	maviMaske = cv2.inRange(rgb, alt_mavi, ust_mavi)
	cv2.imshow("Goruntu2",rgb)
	cv2.imshow("mavi2",maviMaske)
    	
	cv2.waitKey(3)
		
	rate.sleep()
	
	
def scan_callback(msg):
	min_front = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
	min_right = min(msg.ranges[300:345])
	min_left = min(msg.ranges[15:60])
	cmd_vel_pub = rospy.Publisher('/tb3_1/cmd_vel',Twist, queue_size = 10)
	command = Twist()
	rate = rospy.Rate(10)
	global durak
	global near_wall
		
	if(near_wall == 0): #1
		print("robot2: Duvar aranıyor.")
		if(min_front > 0.3 and min_right > 0.2 and min_left > 0.2):    
		    command.angular.z = -0.05    
		    command.linear.x = 0.15
		    
		elif(min_left < 0.2):           
		    near_wall = 1       
		               
		else:
		    command.angular.z = -0.25   
		    command.linear.x = 0.0

		cmd_vel_pub.publish(command)
        
	else:   
		if(min_front > 0.3): #2
			if(min_left < 0.18):    #3
				print("robot2: Range: {:.2f}m - çok yakın. sağa dön.".format(min_left))
				command.angular.z = -1.2
				command.linear.x = 0.15
			elif(min_left > 0.2):  #4
				print("robot2: Range: {:.2f}m - duvar takibi; sola dön.".format(min_left))
				command.angular.z = 1.2
				command.linear.x = 0.15
			#else:
				#print("Range: {:.2f}m - duvar takibi; sağa dön.".format(min_left))
				#command.angular.z = -1.2
				#command.linear.x = 0.15
                
		else:
			print("robot2: önde engel var sağa dön.")
			command.angular.z = -1.0
			command.linear.x = 0.0
			
			
		cmd_vel_pub.publish(command)
	rate.sleep()
        
		




if __name__ == '__main__':
	try:
		rospy.init_node('turtlebot_control', anonymous=True)
		
		rospy.Subscriber('/tb3_1/scan', LaserScan, scan_callback)
		rospy.Subscriber("/tb3_1/camera/image", numpy_msg(Image), imageCallback)
		rospy.spin()
        
       
	except rospy.ROSInterruptException: 
		# clears all window buffers
		pass
