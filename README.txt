Humanoid Robots
Daniel Hong - sh3266
Antong Liu - al3606
Calvin Li - ctl2124

Homework 1
Due Feb. 15, 2017

Part 1
Robot Properties in RViz
- Base Link
- Head Camera Link
- Elbows Link
- Gripper Link
- Bellows Link

Part 2
Least number of navigation goals for 90% environment build:
probably 6
roslaunch fetch_gazebo playground.launch
roslaunch fetch_gazebo_demo fetch_nav.launch map_file:=EMPTY
roslaunch fetch_navigation build_map.launch
rosrun rviz rviz

Part 3
graspable_body_id: 0
    pose: 
      position: 
        x: 0.0599140508077
        y: -0.00174304206426
        z: 0.00234023700216
      orientation: 
        x: -0.141152579828
        y: -0.0348242394185
        z: 0.989104921633
        w: 0.0231230525211
    dofs: [0.0, 1.05218428571138, 0.9603092857113797, 0.9534342857113798]
    epsilon_quality: -1.0
    volume_quality: 0.00152746848122
energies
