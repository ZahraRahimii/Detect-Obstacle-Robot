# Detect Obstacle Robot

To detect obstacle in [the detect_obstacles.world]() we need two nodes:

## Sensor Node: 
This node should read the distance of the robot to the obstacles using the LiDAR sensor and the LaserScan topic and always send the details of the closest obstacle, including the distance and angle, on the ClosestObstacle topic. This topic also needs a [custom message]().

## Control Node: 
This node controls the movement of the robot. Also, we have forward movement and rotation. The robot must always move forward so that its distance from the obstacle is less than 2 meters. Then turn in the right direction until the obstacle is right behind it. Then continue moving straight. 
