# DronTello

	Ejecutar Tello:

	-Terminal 1:

cd
source tello/bin/activate
cd DronTello/Esquinas/
python3 nodoImagenPuertas6.py 


	Terminal 2:

cd
source tello/bin/activate
cd DronTello/TopicsFalsos/
python3 fake_depth_publisher.py


	Terminal 3:

cd
source tello/bin/activate
cd DronTello/ModuloNodos/
python3 nodoTello.py


	Terminal 4:

cd
cd ORB-SLAM3-ROS2-Docker/
sudo docker compose run --rm orb_slam3_22_humble
ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py


	Terminal 5:
	
cd
cd ORB-SLAM3-ROS2-Docker/
sudo docker ps
sudo docker exec -it orb-slam3-ros2-docker-orb_slam3_22_humble-run-{ID} bash
rviz2

	
	
	
	
	
	


