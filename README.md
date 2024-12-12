# Romi-Term-Project
Cal Poly ME 405 Term Project 0x02

Hardware Components:
- Complete Polulu Romi chassis with gearmotors, encoders, and power distribution board
- Nucleo L476 microcontroller
- BNO055 9 DOF IMU, or similar
- Polulu QTR-8RC reflectance sensor array (any array is possible, analog recommended, tweak normalization accordingly)
- 3x bump switch
- HC06 serial Bluetooth module
- Optional "Shoe of Brian" expansion board
When assembling the robot, we recommend mounting the sensor in front of the robot rather than underneath. This improved line following performance, and we used this same mount to secure the bump sensors. The IMU can be screwed to the chassis with standoffs, and the HC06 may require a custom mount as most seem to lack screw holes.


The objective of this code is to use closed loop feedback to guide a 2 wheel Romi robot along a path. The path is specified by a black line, and includes a physical obstacle, pictured below.

![image](https://github.com/user-attachments/assets/8bac884b-ceda-4239-b596-dd8a13399230)

A light sensor array is used to determine the location of the line. By normalizing sensor data, a centroid of reflectance can be obtained, which guides the robot's turns. When the bump sensors on the robot are activated, this triggers an interrupt to route a predetermined square path around the obstacle. 

