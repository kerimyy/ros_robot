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
	cv2.imshow("Goruntu1",rgb)
	cv2.imshow("mavi1",maviMaske)
    	
	cv2.waitKey(3)
		
	rate.sleep()
	
	
def scan_callback(msg):
	min_front = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
	min_right = min(msg.ranges[300:345])
	min_left = min(msg.ranges[15:60])
	cmd_vel_pub = rospy.Publisher('/tb3_0/cmd_vel',Twist, queue_size = 10) #tb3_0 robotu için
	command = Twist()
	rate = rospy.Rate(10)
	global durak
	global near_wall
		
	if(near_wall == 0): # Duvardan uzak olma durumu 
		print("robot1: Duvar aranıyor.")
		if(min_front > 0.3 and min_right > 0.2 and min_left > 0.2): #önde veya yanlarda bir engel olmama durumu
		    command.angular.z = -0.05    
		    command.linear.x = 0.15
		    
		elif(min_left < 0.2): # sol taraftan duvara yakınlık olursa duvar takibine geçilir        
		    near_wall = 1       
		              
		else: # öne engel çıkarsa veya sağ tarafta kalırsa duvarı sola almak için sağa dönüş
		    command.angular.z = -0.25   
		    command.linear.x = 0.0

		cmd_vel_pub.publish(command)
        
	else:   # sol tarafta duvar tespiti olduktan sonra durumu
		if(min_front > 0.3): #önde engel olmama durumu
			if(min_left < 0.18):    
				print("robot1: Range: {:.2f}m - çok yakın. sağa dönülüyor.".format(min_left))
				command.angular.z = -1.2
				command.linear.x = 0.15
			elif(min_left > 0.2):  #burda robor left range 0.18 ile 0.2 arasında tutulur
				print("robot1: Range: {:.2f}m - duvar takibi; sola dönülüyor.".format(min_left))
				command.angular.z = 1.2
				command.linear.x = 0.15
                
		else: # önde engel olma durumu
			print("robot1: önde engel var sağa dönülüyor.")
			command.angular.z = -1.0
			command.linear.x = 0.0
			
			
		cmd_vel_pub.publish(command)
	rate.sleep()
        
		



if __name__ == '__main__':
	try:
		rospy.init_node('turtlebot_control', anonymous=True)
		restartSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
		restartSimulation()
		rospy.Subscriber('/tb3_0/scan', LaserScan, scan_callback)
		rospy.Subscriber("/tb3_0/camera/image", numpy_msg(Image), imageCallback)
		rospy.spin()
        
       
	except rospy.ROSInterruptException: 
		# clears all window buffers
		pass
