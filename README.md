# Loiter Munition and Precision Dive

The objective of this project was to enable a precise kamekaze dive on a user-selected target from real time camera feedback.
After the target was selected a precise target dive was executed with real time trajectory adjustment from the camera feedback mounted at the front of the fixed-wing plane.

![video](https://github.com/saksham18kukreja/loiter_munition/blob/main/media/dive.gif)

In the video the target was selected as the origin and a dive was conducted (target was missed due to lack of real time adjustment in trajectory)

## Fallback
![video](https://github.com/saksham18kukreja/loiter_munition/blob/main/media/recovery.gif)

A fallback mechanism was also designed to abort the mission and continue flying.

## Contribution
1. Designed the trajectory generation algorithm towards the target.
2. Integrated the code as a new flight mode with the PX4 autopilot stack
3. Devised the recovery flight mode.
4. Designed the object tracking module using OpenCV BOOSTING library.
5. Sytem design and integration of flight modules.


## Tools Used
1. Python
2. PX4 autopilot stack
3. Mavros and ROS
4. OpenCV
