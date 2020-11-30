*Guide for sensor integration*
-run the respective sensor launch files to start the sensor
-sudo chmod 666 /dev/(sensor_port) to activate the port
-Calibrate the position and orientation of the sensors according to the hardware setup using the transform messages published in the launch files.
-Comment the transform publishing line while mapping using Hector SLAM.

