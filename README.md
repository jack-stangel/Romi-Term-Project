# Romi-Term-Project
Cal Poly ME 405 Term Project 0x02

**Hardware Components**

- Complete Polulu Romi chassis with gearmotors, encoders, and power distribution board
- Nucleo L476 microcontroller
- BNO055 9 DOF IMU, or similar
- Polulu QTR-8RC reflectance sensor array (any array is possible, analog recommended, tweak normalization accordingly)
- 3x bump switch
- HC06 serial Bluetooth module
- Optional "Shoe of Brian" expansion board

**Robot Assembly**

When assembling the robot, we recommend mounting the sensor in front of the robot rather than underneath. This improved line following performance, and we used this same mount to secure the bump sensors. The IMU can be screwed to the chassis with standoffs, and the HC06 may require a custom mount as most seem to lack screw holes.

![1000005427](https://github.com/user-attachments/assets/c0a46626-76c4-4cff-a0f8-5776d340aaf9)

**Objective**

The objective of this code is to use closed loop feedback to guide a 2 wheel Romi robot along a path. The path is specified by a black line, and includes a physical obstacle, pictured below.

![image](https://github.com/user-attachments/assets/8bac884b-ceda-4239-b596-dd8a13399230)

A light sensor array is used to determine the location of the line. By normalizing sensor data, a centroid of reflectance can be obtained, which guides the robot's turns. When the bump sensors on the robot are activated, this triggers an interrupt to route a predetermined square path around the obstacle. 

**Control Theory**

The control theory surrounding the movement of Romi is depicted in the block diagram below.

![image](https://github.com/user-attachments/assets/9319c3a9-f0e9-4254-ac34-a84a8b26d6f2)

This control loop utilizes proportional and derivative gain for the line sensing task, and will return a speed correction factor that gets placed in a proportional and integral controller for the motor speeds. The reasoning behind excluding the integral gain in line sensing is due to the huge error and discrepancies that this caused when running our tasks. Furthermore, we found that an increase in derivative gain provided our Romi with the most guidance when dealing with very tight turns such as after the dashed line sequence. Our motor control loop was left unchanged throughout the term as we were satisfied with the motor performances with the corresponding values. The overall control theory holds true at every point during the obstacle, except during the bump sensing as that triggers an interrupt as mentioned.
