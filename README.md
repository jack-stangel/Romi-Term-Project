# Romi-Term-Project
Cal Poly ME 405 Term Project 0x02: mecha13

**Hardware Components**

- Complete Polulu Romi chassis with gearmotors, encoders, and power distribution board
- Nucleo L476 microcontroller
- BNO055 9 DOF IMU, or similar
- Polulu QTR-8RC reflectance sensor array (any array is possible, analog recommended, tweak normalization accordingly)
- Acryllic Standoff Piece
- 3x bump switch
- HC06 serial Bluetooth module
- Optional "Shoe of Brian" expansion board

**Robot Assembly**

When assembling the robot, we recommend mounting the sensor in front of the robot rather than underneath. This improved line following performance, and we used this same mount to secure the bump sensors. The IMU can be screwed to the chassis with standoffs, and the HC06 may require a custom mount as most seem to lack screw holes. The Nucleo and Shoe of Brian is mounted via standoffs and a piece of laser-cut acryllic.

![1000005427](https://github.com/user-attachments/assets/c0a46626-76c4-4cff-a0f8-5776d340aaf9)

**Objective**

The objective of this code is to use closed loop feedback to guide a 2 wheel Romi robot along a path. The path is specified by a black line, and includes a physical obstacle, pictured below.

![image](https://github.com/user-attachments/assets/8bac884b-ceda-4239-b596-dd8a13399230)

A light sensor array is used to determine the location of the line. By normalizing sensor data, a centroid of reflectance can be obtained, which guides the robot's turns. When the bump sensors on the robot are activated, this triggers an interrupt to route a predetermined square path around the obstacle. 

**Control Theory**

The control theory surrounding the movement of Romi is depicted in the block diagram below.

![image](https://github.com/user-attachments/assets/9319c3a9-f0e9-4254-ac34-a84a8b26d6f2)

This control loop utilizes proportional and derivative gain for the line sensing task, and will return a speed correction factor that gets placed in a proportional and integral controller for the motor speeds. The reasoning behind excluding the integral gain in line sensing is due to the huge error and discrepancies that this caused when running our tasks. Furthermore, we found that an increase in derivative gain provided our Romi with the most guidance when dealing with very tight turns such as after the dashed line sequence. Our motor control loop was left unchanged throughout the term as we were satisfied with the motor performances with the corresponding values. The overall control theory holds true at every point during the obstacle, except during the bump sensing as that triggers an interrupt as mentioned.

**Task Diagram**

A high-level task diagram is shown below for the main program. As seen, variables and other parameters are shared and interchanged between various tasks. Of note is the no shared variables between the IMU and bump tasks. Since the IMU wasn't utilized in this term project, it didn't have any variables and was set to a lower priority. Additionally, since the bump task was set as an interrupt and hard coded to perform a sequence, no variables were shared for that task and was similarly set to a lower priority. Since our printing task varied based on what we were testing at the time, this was set a lower priority as well.

![image](https://github.com/user-attachments/assets/5135da5d-a0ef-4bba-b5a0-20502286f337)

**Motor Tasks: Finite State Machines and Code Analysis**

The finite state machines for both motors are shown below. These tasks are identical and only vary in that one motor is being called instead of the other.

![image](https://github.com/user-attachments/assets/4ac81147-3ec5-4f15-a0a8-2cd67a45c08f)
![image](https://github.com/user-attachments/assets/06e0cdf2-2bf6-4d17-b813-f637ef2d78de)

As noted within the finite state machines, the motors undergo an initialization process where they are enabled and configured and start a timer before the "while True" portion of our task. This initialization is performed with the use of the romi_driver.py and encoder_driver.py files. This ensured that these only occur once and not repeated as the scheduler is running, as well as properly initialized all motor and encoder objects. The task then checks for the bump flags that will be set during the event of a bump. In the event of a bump, the motor duty cycles and adjusted velocities in the share get placed with zeros. This was done as during testing we found that this caused some issues with our closed loop feedback at the conclusion of the bump sequence. Hard coding these values minimized the error that was seen after the conclusion of the bump sequence. For the duration of the bump flag, the motor tasks would yield and allow the other tasks to run. If no bumps were pressed, then the code will proceed by reading the encoders and calculating the velocities that the wheels experience. This was in turn placed in the closed loop PI feedback using the closed_loop_driver.py file, with the inputs being the measured velocity and reference velocity set in the line following task, and the output being the PWM to supply the motors with to maintain the velocity. The velocities then get placed in a share and the task gets yielded to allow the other tasks to run. As a safety precaution, we also decided to stop all motors during a keyboard interrupt as we found issues where they would keep spinning. This would then be repeated for the duration of the program. Reflecting on our code for this task, we would've liked to further tune the motors by adding derivative control (which already has the skeleton to begin testing), however we prioritized other portions of this project such as line following. Nonetheless, the motor tasks were among our best programmed tasks for this project.

