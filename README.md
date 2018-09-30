# Description of the Scara robot package

There`re two posibilities to start the project:
1. ``` roslaunch scara_robot run_gazebo.launch``` - which starts robot in gazebo
2. ```  roslaunch scara_robot run_rviz.launch``` - which starts robot in rviz

And to run circles we should run ```rosrun scara_robot main.py``` 
which executes 2 circles: firstly in YZ plane and nextly in XY plane.


The URDF tree is located in ```scara_robot/urdf/SCARA.pdf```.
