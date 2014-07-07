# Perception3d



## Calibration Steps of Jaco with ARTags
1. Create an approximate transformation between the kinect and jaco (in vision.launch, named 'jaco_kinect')
2. Launch jaco_calibration.launch
3. Open Rviz
4. Make sure the AR Marker can be seen by the kinect by moving the arm and wrist of Jaco
5. Check that the tf 'AR_OBJECT' exists (if the marker is not detected, the tf will not exist)
6. Copy and paste the resulted transform (printed in the terminal) in vision.launch  





## How to use the package

    roslaunch perception3d vision.launch
    rosrun perception3d perception
