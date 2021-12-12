#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.srv import *
from std_srvs.srv import *
from turtlesim.msg import Pose
import math
import sys
import time

PI = math.pi
def dunya_hareket(yon, distance,isForward): # U ekseninde hareketler için R ekseninden dönüştüren fonksiyon
	konum = rospy.wait_for_message('/turtle1/pose', Pose, timeout=None) # turtle1 in konum bilgileri alınır
	ang = konum.theta/(2*math.pi)*360 # konum.theta ya radian işleminin tersi uygulanarak açısı öğrenilir
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
	vel_msg = Twist()
	speed = 2
	i = -1 # eksili değer kontrolü için 
	if isForward:
		i = 1
	if yon: # yon parametresi 1 se y ekseni, 0 sa x ekseni 
		vel_msg.linear.x = abs(speed*math.sin(math.radians(ang)))
		vel_msg.linear.y = i*abs(speed*math.cos(math.radians(ang)))
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
		
	else:
		vel_msg.linear.x = i*abs(speed*math.cos(math.radians(ang)))
		vel_msg.linear.y = i*-abs(speed*math.sin(math.radians(ang)))
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
        
	if not rospy.is_shutdown(): # hareket işlemleri ve hedefe ulaşma kontrolleri
		t0 = float(rospy.Time.now().to_sec())
		current_distance = 0
		#robotu belirlenmis mesafeye goturme islemi
		while(float(current_distance) < float(distance)):
			velocity_publisher.publish(vel_msg)
			#gercek zamanin konuma cevrilmesi
			t1=float(rospy.Time.now().to_sec())
			current_distance= float(speed*(t1-t0))
		#robotun hizi sifirlandi
		vel_msg.linear.x = 0
		#robot durduruldu
		velocity_publisher.publish(vel_msg)
		
def moveToTarget(distance, isForward): # R ye göre n ekseninde hareket fonksiyonu
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	vel_msg = Twist()
	speed = 2

	if isForward:
		vel_msg.linear.x = abs(speed)
	else:S
		vel_msg.linear.x = -abs(speed)
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
        
	if not rospy.is_shutdown():
		t0 = float(rospy.Time.now().to_sec())
		current_distance = 0

		#robotu belirlenmis mesafeye goturme islemi
		while(float(current_distance) < float(distance)):
			velocity_publisher.publish(vel_msg)
			#gercek zamanin konuma cevrilmesi
			t1=float(rospy.Time.now().to_sec())
			current_distance= float(speed*(t1-t0))
		#robotun hizi sifirlandi
		vel_msg.linear.x = 0
		#robot durduruldu
		velocity_publisher.publish(vel_msg)
        
def rotate(angle, clockwise): # R ye göre a ekseninde dönme fonksiyonu
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = 20 #aci/saniye

    #dereceyi radyana cevirme
    angular_speed = speed*2*PI/360
    relative_angle = float(float(angle)*2*PI/360)

    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # robot yonu kontrolu (saat yonu- saat yonu tersi)
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
   
    t0 = float(rospy.Time.now().to_sec())
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = float(rospy.Time.now().to_sec())
        current_angle = float(angular_speed*(t1-t0))


    #robot durduruldu.
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def poseCallback(data): # info için
    rospy.loginfo("pose x: " + str(data.x) + "pose y: " + str(data.y) + "pose theta: " + str(data.theta/(2*PI)*360))
    
if __name__ == '__main__':
	try:
		#ros icin gerekli olan temel fonksiyonlarin olusturulmasi
		rospy.init_node('ninja_kaplumbaalar', anonymous=True)
		killTurtle = rospy.ServiceProxy('/kill', Kill)
		spawnTurtle = rospy.ServiceProxy('/spawn', Spawn)
		clearStage = rospy.ServiceProxy('/clear', Empty)
		
		while True:
			S
			
			killTurtle("turtle1")
			spawnTurtle(1,1,0,"turtle1") #turtle1 i istenilen konumda ve açıda oluşturulur
			
			dunya_hareket(1,2,1) # U ya göre y ekseninde 2 birim hareket et
			
			rotate(30, 0) # a ekseni etrafında 30 derece dön 
			
			dunya_hareket(0,1,1) # U ya göre x ekseninde 1 birim hareket et

			rotate(60, 1) # R ye göre a ekseninde -60 derece dön

			dunya_hareket(1,1,0) # U ya göre y ekseninde -1 birim hareket et

			rotate(60, 1) # R ye göre a ekseninde -60 derece dön

			moveToTarget(1,1) # n ekseninde 1 birim hareket et
			
			
			
			
			
			
			
		
				
		
			
				
	except rospy.ROSInterruptException: pass
