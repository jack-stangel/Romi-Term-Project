# Romi-Term-Project
Cal Poly ME 405 Term Project 0x02: mecha13

**Hardware Components**

- Complete Polulu Romi chassis with gearmotors, encoders, and power distribution board
- Nucleo L476 microcontroller
- BNO055 9 DOF IMU, or similar
- Polulu QTR-MD-06A reflectance sensor array (any array is possible, analog recommended, tweak normalization accordingly)
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

**Romi Trial Video**

The link below will demonstrate the full video of our Romi navigating the path and obstacle.
https://drive.google.com/file/d/1LN1t5X5QpA5D_mpJcGSsuaPToGwDRCUk/view?usp=drive_link

**Control Theory**

The control theory surrounding the movement of Romi is depicted in the block diagram below.

![image](https://github.com/user-attachments/assets/9319c3a9-f0e9-4254-ac34-a84a8b26d6f2)

This control loop utilizes proportional and derivative gain for the line sensing task, and will return a speed correction factor that gets placed in a proportional and integral controller for the motor speeds. The reasoning behind excluding the integral gain in line sensing is due to the huge error and discrepancies that this caused when running our tasks. Furthermore, we found that an increase in derivative gain provided our Romi with the most guidance when dealing with very tight turns such as after the dashed line sequence. Our motor control loop was left unchanged throughout the term as we were satisfied with the motor performances with the corresponding values. The overall control theory holds true at every point during the obstacle, except during the bump sensing as that triggers an interrupt as mentioned.

**Task Diagram**

A high-level task diagram is shown below for the main program. As seen, variables and other parameters are shared and interchanged between various tasks. Of note is the no shared variables between the IMU and bump tasks. Since the IMU wasn't utilized in this term project, it didn't have any variables and was set to a lower priority. Additionally, since the bump task was set as an interrupt and hard coded to perform a sequence, no variables were shared for that task and was similarly set to a lower priority. Since our printing task varied based on what we were testing at the time, this was set a lower priority as well. In our main program, these tasks and parameters were shared using the task_share.py file, and the scheduler allowed cooperative multi-tasking using the cotask.py file 

![image](https://github.com/user-attachments/assets/5135da5d-a0ef-4bba-b5a0-20502286f337)

**Line Following Task: Finite State Machine and Code Analysis**

The finite state machine for the line following task is shown below.

![image](https://github.com/user-attachments/assets/689edf96-07b7-4f7c-87e4-ecb513a40138)

As seen in the finite state machine, the line following task undergoes an initialization that includes a threshold for line detection, the gain values for the closed loop control, as well as the Romi dynamics such as the reference velocity. Furthermore, a timer known as "sense_time" is initialized that will drive most of the program to its desired outcomes. Starting in a sequential order, while sense_time is below 2 seconds, the task will not begin line sensing. This is to avoid confusion near the start line where a fully black line is present, which testing proved to hinder line sensing abilities. The robot will move forward at the specified linear velocity for these 2 seconds, at which point it will begin to sense the lines. The line sensing occurs in the QTR_driver.py file, where an analog output is read and normalized, and then placed under a threshold. This threshold will set the detection limit towards what is or isn't a line. This threshold was fairly simple to tune, and the output of this thresholded data is a string of binary numbers that indicate where a line is, where "1" means that sensor reads a line, and a "0" doesn't. These values are then weighted based on the geometry of the line sensor and will output a centroid will serve as the main control loop component. If directly on the line, the centroid reads zero, with a symmetric scale on opposite ends to serve as correction methods. If no centroid is read, meaning no line detected, the robot will gradually come to a stop to prevent any stray errors or movements until line sensing again. If there is a valid centroid, it is then placed in a closed loop feedback using proportional and derivative gains to help keep the robot centered. Within this feedback loop, an "if" statement reads the accumulated error and sets different gain values and velocities based on higher errors. This was done for our sharp turns, as we found that slower speeds with larger derivative gain helped in that aspect, which allowed for a dynamic gain control loop. The output of the feedback is a velocity correction factor that is based on Romi's dynamics. The velocities get placed in the share and the task is yielded. This is repeated until sensing_time reaches 1 minute. Our personal time trials found that Romi completes the course exactly in 1 minute, at which point the task no longer is line following, and instead a hard-code is set that takes Romi back to start from the finish. At the conclusion, the motors are disabled, thus ending the program. Reflecting on this task, a lot could've been done to improve its performance, with the main takeaway being to improve and better tune the gains for the line following control loop. The initial gains were hard coded and allowed for a very shaky Romi during the entirety of our project. If we had more time, we would've spent more on better tuning these values to ensure smoother line following. The same can be said about our weighing of sensors within the driver as this also had an effect on how our Romi moved. Furthermore, the handling of our finish condition could've been better approached by having a condition with the solid finish line, rather than using a timer for the states of this task. Though in our video it shows that it can be a valid approach, it is by no means optimal and would handle a lot better with other feedback such as the IMU. Lastly, we would've handled the placement of our line sensor a lot differently than what it was on gameday. As mentioned above, a 3D printed mount was made to accomodate a QTR8RC line sensor near the front of Romi, which would've allowed for faster and wider detection. However, this sensor unfortunately malfunctioned the day before and we had to improvise and use a spare analog sensor and mounted it under Romi and being held up purely by the wires. Though the analog was easier to work with in code, it was not the intended placement and possibly affected our results. Though this task was complex and not perfectly tuned, it still ultimately worked and was consistent at following the line as intended.

**Bump Task: Finite State Machine and Code Analysis**

The finite state machine for the bump sensing task is shown below.

![image](https://github.com/user-attachments/assets/72d5bb9a-487e-4986-9829-a03c10b06c2d)

The bump task undergoes a brief initialization where the bump flag is initially set to "False" and then continously checks if a bump has been detected. The bumps were configured using external interrupts and a pull down resistor to keep an active high state for the bumps. As mentioned, the task continously yields unless a bump is detected, at which point this task overrides the others due to the interrupt and commences the obstacle navigation. Since our obstacle is a cardboard cube, hard code was set in place to directly modify the PWM's of each motor for a specific amount of time to navigate around this obstacle. At the conclusion, the program reset all motor velocities and control loops to prevent any large errors from arising due to the interrupt. The bump flag is then lowered and all tasks resume as normal, with Romi looking for the line directly afterwards. Reflecting on this task, though it seemed easy to hard code the manuevers, it proved to be ineffective due to the battery drainage at certain periods of time. With fully charged batteries, some of the manuevers didn't require as much time as they did before the batteries were charged, and proved to be inconsistent as the testing went on. We would change the manuevering mechanism and instead use yaw rate control to perform a circle that begins to line sensing after a certain heading had been reached. This would've prevented the battery drainage issues and would've also provided a rather cleaner transition to line following.

**Motor Tasks: Finite State Machines and Code Analysis**

The finite state machines for both motors are shown below. These tasks are identical and only vary in that one motor is being called instead of the other.

![image](https://github.com/user-attachments/assets/4ac81147-3ec5-4f15-a0a8-2cd67a45c08f)
![image](https://github.com/user-attachments/assets/06e0cdf2-2bf6-4d17-b813-f637ef2d78de)

As noted within the finite state machines, the motors undergo an initialization process where they are enabled and configured and start a timer before the "while True" portion of our task. This initialization is performed with the use of the romi_driver.py and encoder_driver.py files. This ensured that these only occur once and not repeated as the scheduler is running, as well as properly initialized all motor and encoder objects. The task then checks for the bump flags that will be set during the event of a bump. In the event of a bump, the motor duty cycles and adjusted velocities in the share get placed with zeros. This was done as during testing we found that this caused some issues with our closed loop feedback at the conclusion of the bump sequence. Hard coding these values minimized the error that was seen after the conclusion of the bump sequence. For the duration of the bump flag, the motor tasks would yield and allow the other tasks to run. If no bumps were pressed, then the code will proceed by reading the encoders and calculating the velocities that the wheels experience. This was in turn placed in the closed loop PI feedback using the closed_loop_driver.py file, with the inputs being the measured velocity and reference velocity set in the line following task, and the output being the PWM to supply the motors with to maintain the velocity. The velocities then get placed in a share and the task gets yielded to allow the other tasks to run. As a safety precaution, we also decided to stop all motors during a keyboard interrupt as we found issues where they would keep spinning. This would then be repeated for the duration of the program. Reflecting on our code for this task, we would've liked to further tune the motors by adding derivative control (which already has the skeleton to begin testing), however we prioritized other portions of this project such as line following. Nonetheless, the motor tasks were among our best programmed tasks for this project.

**IMU Task: Finite State Machine and Code Analysis**

The finite state machine for the IMU is displayed below.

![image](https://github.com/user-attachments/assets/b6f008ff-9fcb-4c76-9de6-5d505efcff89)

This task initializes the IMU to fusion mode via an IMU object that is created from the IMU_driver.py file. The object is able to retrieve calibration data, heading, and yaw rate data from the IMU. In this task, upon the initialization, the data gets read and placed in shared variables and then yielded. As mentioned previously, the IMU was not utilized for this project so the code for this task was rather simple. Reflecting on this task and the IMU in general, we would've liked to use the IMU to help our Romi go from the finish line to the start line again via a control loop. This would have been performed by utilizing the heading in our program and reference for the loop. We also briefly explored yaw rate control for this project, however given the time constraints and other priorities decided against it. Nonetheless, we feel as though having the IMU would've largely benefitted us and we would highly encourage some form of IMU incorporation for future projects such as this one.

**Printing Task: Finite State Machine and Code Analysis**

The finite state machine for the printing task is displayed below.

![image](https://github.com/user-attachments/assets/d237f82b-8b88-4354-806e-cac43a8aa170)

This task would initialize by starting a timer to allow the times to be printed with any data. The task would then read any relevant shared values that were necessary to be printed. In our final code, we opted to print out the velocities of both wheels and the total linear velocity, however this varied throughout the project as we also utilized this task to print out our line sensor readings and values, program flow, as well as other relevant debugging parameters. This task would print a table that was formatted with the help of ChatGPT for our data printing. This task would then yield, and be repeated as the scheduler ran. Reflecting on this task, we had the benefit of having the HC06 Bluetooth Module which allowed us to run our program and record data with Romi untethered. This proved to be useful when debugging certain parameters, mainly with line sensing and we view this task as one of the more underrated and valuable ones in this project.

The wiring schematic for our Romi is shown below. NOTE: Shoe of Brian expansion board and Romi power distribution board are not included. Only final Nucleo pins are shown.

![Wiring Diagram](https://github.com/user-attachments/assets/29183009-962f-4119-a483-ffb5787614be)
