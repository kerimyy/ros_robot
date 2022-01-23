#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import imutils
from collections import deque
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

from std_srvs.srv import *
from math import pow,atan2,sqrt
import sys
import time
PI = 3.1415926535897

def pidKontrol(hedef, sapma):
	# PID KONTROL KODUNU YAZ
	sinyal = 0.15
	return sinyal
	
def odomCallback(data):
	rate = rospy.Rate(2)
	#pose veya twist. Pose ile aci quaternion oldugundan twist secildi.
	x = data.twist.twist.linear.x
	y = data.twist.twist.linear.y
	z = data.twist.twist.linear.z
	ax = data.twist.twist.angular.x
	ay = data.twist.twist.angular.y
	az = data.twist.twist.angular.z
	#Konum bilgisini yazdir
	print("x: %.2f y: %.2f z: %.2f" % (x,y,z))
	print("ax: %.2f ay: %.2f az: %.2f" % (ax,ay,az))
	
	rate.sleep() 
	
def scan_callback(msg):
    FRONT = min(min(msg.ranges[0:5]), min(msg.ranges[354:359]))
    RIGHT = min(msg.ranges[300:345])
    LEFT = min(msg.ranges[15:60])
    
    rate = rospy.Rate(2)
    
    min_range = msg.range_min
    max_range = msg.range_max
    min_angle = msg.angle_min
    max_angle = msg.angle_max
    angle_incr = msg.angle_increment 
    print("Scanner data FRONT: %.2f RIGHT: %.2f LEFT : %.2f" % (FRONT,RIGHT,LEFT))
    print("min_range: %.2f max_range: %.2f" % (min_range,max_range))
    print("min_angle: %.2f max_angle: %.2f angle_incr: %.2f" % (min_angle,max_angle,angle_incr))
    
    rate.sleep()  
    
def imageCallback(data):
	frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    Greenmask = cv2.inRange(hsv, (20, 150, 150), (255, 145, 120))
    Greenmask = cv2.erode(Greenmask, None, iterations=2)
    Greenmask = cv2.dilate(Greenmask, None, iterations=2)

    # getting the range of blue color in frame
    
    
    
    # applying HoughCircles
    circles = cv2.HoughCircles(Greenmask, cv2.HOUGH_GRADIENT, 1, Greenmask.shape[0] / 8, param1=100, param2=18, minRadius=5, maxRadius=60)

    if circles is not None:
        circles = np.uint16(np.around(circles))


        for i in circles[0,:]:
        # draw the outer circle
            if (i is not None):
                cv2.circle(circles,(i[0],i[1]),i[2],(0,255,0),2) # x,y,circle center
                # draw the center of the circle
                cv2.circle(circles,(i[0],i[1]),2,(0,0,255),3)
                center = (i[0],i[1])
                if(i[2] != 0):
                    cv2.circle(frame, (center[0], center[1]), i[2], (0, 0, 255), 2)
                    cv2.circle(frame, center, 5, (0, 255, 255), -1)
                    points.appendleft(center) # format is (center[0] == X,center[1] == Y)
                    cv2.putText(frame, "Circle is founded", (10, 700), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    print("Nesne Bulundu")
                else:
                    cv2.putText(frame, "Error no circel Found", (10, 700), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    print("Nesne Bulunamadı...")
            else:
                cv2.putText(frame, "Error no circel Found", (10, 700), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    
    
    
    
        DxCount = 0.0
        DyCount = 0.0
    
        for i in np.arange(1, len(points)):
            if points[i - 1] is None or points[i] is None:
                continue
            
            if counter >= 10 and i == 1:
                DxCount = float(points[i][0]) - 320.0
                DyCount = 170 - float(points[i][1])
    
                cv2.line(frame, points[i - 1], (320, 170), (0, 0, 255), 5)
    
                if (DxCount < 0):
                    Direction1 = "Need to go Left"
    
                if (DxCount > 0):
                    Direction1 = "Need to go Right"
    
                if (DyCount < 0):
                    Direction2 = "Need to go Up"
    
                if (DyCount > 0):
                    Direction2 = "Need to go Down"
    
                if (int(points[i][0]) - 320) * 2 + (int(points[i][1]) - 170) * 2 < (TargetCircleRaduis) ** 2:
                    InsideCircle = True
                else:
    
                    InsideCircle = False
    
        cv2.putText(frame, "Inside circel" + str(InsideCircle), (10, 120), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
    
        print("X: " + str(Direction1) + "      Y:" + str(Direction2))

        if(InsideCircle == true):
            print("ortaladı")
	
	# keeps the window open until a key is pressed
	cv2.waitKey(3)
	
	rate.sleep()
 
if __name__ == '__main__':
	try:
		rospy.init_node('turtlebot_control', anonymous=True)
		restartSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
		restartSimulation()
		rospy.Subscriber("/camera/image", numpy_msg(Image), imageCallback)
		rospy.Subscriber('/odom', Odometry, odomCallback)
		rospy.Subscriber('/scan', LaserScan, scan_callback)
		rospy.spin()
		
		# Ornek Twist publisher ve Pose subscriber
		# pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		# rospy.Subscriber('/odom', Odometry, odomCallback)
		# rospy.spin()
		#rate = rospy.Rate(10)
		#rate.sleep()	
		
		# Tekil mesaj okumalarinda wait_for_message fonksiyonu alternatif olabilir.
		#msg = rospy.wait_for_message('/odom', Odometry, timeout=None)
		#odomCallback(msg)
				
	except rospy.ROSInterruptException: 
		# clears all window buffers
		cv2.destroyAllWindows()
		pass
