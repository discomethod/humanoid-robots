# Team 4 Humanoid Robotics Project

This repo was forked from the HumanoidRobotics/fetch_gazebo repository

Task 1:
Use vision system to detect object, moving robot along the table until the object is detected, then grasp the object and move around the table in order to deliver to the customer.

The code for this is located at fetch_waiter/scripts/visual.py

Task 2:
Given location of object, use precomputed vectors file to find best position for robot to go to
and grasp the object the fastest, then move around the table in order to deliver to the customer.

The code for this is located at fetch_waiter/scripts/waiter.py


Task 3:
Given location of object, use precomputed vectors file to find best position for robot to go to
and grasp the object the fastest, then use Dijkstra's algorithm on a visibility graph of grown
obstacles in order to navigate a cluttered environment to get to the pickup location, as well as
to get to the delivery location.

The code for this is located at fetch_waiter/scripts/better_waiter.py

Made a custom table model in fetch_gazebo/models/table

