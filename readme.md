# Projeyi çalıştırmak için:

/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch 
klasörüne "proje2.launch" dosyası atılmalı

/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds
klasörüne "world6.world" dosyası atılmalı

/catkin_ws/src/turtlebot3_camera/scripts
klasörüne labirent2.py ve labirent2_2.py dosyaları atılmalı

Adımlar:
1.roscore çalıştırılacak : "roscore"
2.yeni bir terminalde : "roslaunch turtlebot3_gazebo proje2.launch" komutu çalıştırılmalı
3.yeni bir terminalde : "rosrun turtlebot3_camera labirent2.py" komutu çalıştırılmalı
4.yeni bir terminalde : "rosrun turtlebot3_camera labirent2_2.py" komutu çalıştırılmalı # ikinci robot için




