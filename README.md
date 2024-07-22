# ALBRT-1 (Autonomous Lidar-Based Robot Teleoperation)
![chasis_arduino_car_4x4 v4 v17](https://github.com/user-attachments/assets/09d2bcde-0feb-41ab-affa-cdde6fe14893)

![IMG_5100](https://github.com/user-attachments/assets/bf01aaaf-b312-403e-9f9d-7036d5887f98)


![ALBRT-v1Explodedviewanimation-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/40d6e79d-3c68-4362-91ed-5f30f59f54f4)




## Getting the LiDAR Running:
(cloned from: https://github.com/babakhani/rplidar_ros2)
```
cd ~/Desktop/ros2_ws # change directory to where your ros 2 work space is

sudo chmod a+rw /dev/ttyUSB0 # modifies the permission of the device file, in this case to be able to read/write to the serial port our lidar is connected to
# sudo chmod 666 /dev/ttyUSB0 # equivalent command to above

source install/setup.bash # configure your current shell/terminal to ROS 2 

#ros2 launch rplidar_ros view_rplidar.launch.py

ros2 launch rplidar_ros rplidar.launch.py # launches without Rviz
```


![ALBRT-1_bb](https://github.com/user-attachments/assets/b09f2987-7b09-4a13-bd09-3a4f2dc0f90e)


![ALBRT-1_schem](https://github.com/user-attachments/assets/e1755fbc-0149-4724-8a98-0188c127f223)
