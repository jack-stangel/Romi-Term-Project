# Importing Necessary Modules 
import pyb
from pyb import Pin, Timer, UART, ExtInt
from time import ticks_ms, ticks_diff
import encoder_driver, romi_driver, IMU_driver, closed_loop_driver, cotask, task_share, QTR_driver

# Bluetooth Initialization
BT_ser = UART(1, 115200)
Pin(Pin.cpu.B6, mode=Pin.ANALOG)
Pin(Pin.cpu.A9, mode=Pin.ALT, alt=7)
pyb.repl_uart(BT_ser)

# Constants
Kp_motor = 3.50
Ki_motor = 2.75
Kd_motor = 0.00
track_width = 5.86  # in
wheel_radius = 1.42  # in

# Bump Sensing Logic
bump_flag = False

def obstacle_nav(left_pwm, right_pwm, time):
    '''!@brief Directly set motor PWM for obstacle navigation.
        @param left_pwm PWM value for the left motor (-100 to 100).
        @param right_pwm PWM value for the right motor (-100 to 100).
        @param time Duration to apply the PWM values (in milliseconds).
    '''
    mot_A.set_duty(left_pwm)
    mot_B.set_duty(right_pwm)
    start_time = ticks_ms()
    while ticks_diff(ticks_ms(), start_time) < time:
        pass  # Wait for the specified time
    mot_A.set_duty(0)
    mot_B.set_duty(0)
    share_adjusted_velocity_A.put(0)
    share_adjusted_velocity_B.put(0)
    mot_A.set_duty(0)
    mot_B.set_duty(0)
    mot_A_control.reset()  # Reset feedback controller for Motor A
    mot_B_control.reset()  # Reset feedback controller for Motor B
    
# Return Logic
return_flag = False
first_time = ticks_ms()

def handle_bump(line):
    global bump_flag
    bump_flag = True
        
# Timer and Hardware Object Initializations
tim_N = Timer(3, period=65535, prescaler=0)
tim_M = Timer(2, period=65535, prescaler=0)
tim_A = Timer(1, freq=20000)
tim_B = Timer(4, freq=20000)

# Encoder Initializations
enc_A = encoder_driver.Encoder(tim_N, Pin.cpu.B4, Pin.cpu.B5)
enc_B = encoder_driver.Encoder(tim_M, Pin.cpu.A0, Pin.cpu.A1)

# Motor Initializations
mot_A = romi_driver.Romi(tim_A, Pin.cpu.B3, Pin.cpu.A7, Pin.cpu.A8)
mot_B = romi_driver.Romi(tim_B, Pin.cpu.C7, Pin.cpu.B10, Pin.cpu.B6)

# Control Loops
mot_A_control = closed_loop_driver.ClosedLoop(Kp_motor, Ki_motor, Kd_motor)
mot_B_control = closed_loop_driver.ClosedLoop(Kp_motor, Ki_motor, Kd_motor)

# Shared resources for storing motor and line data
share_position_A = task_share.Share('f', thread_protect=False, name="Position_A")
share_velocity_A = task_share.Share('f', thread_protect=False, name="Velocity_A")
share_position_B = task_share.Share('f', thread_protect=False, name="Position_B")
share_velocity_B = task_share.Share('f', thread_protect=False, name="Velocity_B")
share_adjusted_velocity_A = task_share.Share('f', thread_protect=False, name="Adjusted_Velocity_A")
share_adjusted_velocity_B = task_share.Share('f', thread_protect=False, name="Adjusted_Velocity_B")
share_heading = task_share.Share('f', thread_protect=False, name="Heading")
share_yaw = task_share.Share('f', thread_protect=False, name = "Yaw")
        
def task_line_following():
    global return_flag
    # Constants
    threshold = 0.85  # Initial threshold for line detection
    Kp_line = 2.65    # Proportional gain for line following
    Ki_line = 0.0     # Integral gain for line following
    Kd_line = 0.76    # Derivative gain for line following
    linear_velocity = 6.25  # Desired robot linear velocity [in/s], was 5.0 and working
    no_line_timeout = 2500  # Maximum time (ms) without detecting a line
    centroid_set = 0.0
    max_integral = 10.0  # Clamp for integral term

    # Variables
    integral_line = 0.0
    last_time = ticks_ms()
    last_line_time = ticks_ms()
    last_error = 0.0
    last_left_velocity = 0.0
    last_right_velocity = 0.0    
    sense_time = ticks_ms()
    while ticks_diff(ticks_ms(), sense_time) < 2000:
        # Keep the motors spinning with default velocities during the delay
        target_angular_velocity = linear_velocity / wheel_radius  # [rad/s]
        share_adjusted_velocity_A.put(target_angular_velocity)
        share_adjusted_velocity_B.put(target_angular_velocity)
        yield 0  # Allow other tasks to run
        
    while True:
        if ticks_diff(ticks_ms(),first_time) > 60000:
            return_flag = True
        if return_flag == 0:
            # Step 1: Read the line position
            sensor_data = QTR_driver.read_and_normalize_data()
            binary_data = QTR_driver.apply_threshold(sensor_data, threshold)
            centroid_offset = QTR_driver.compute_centroid(binary_data)
    
            if centroid_offset is not None:
                # Step 2: Compute error and reset the no-line timer
                error = -(centroid_set - centroid_offset)  # Negative because we want 0 at the center
                last_line_time = ticks_ms()
    
                # Step 3: Calculate correction factor
                current_time = ticks_ms()
                dt = ticks_diff(current_time, last_time) / 1000  # Time step in seconds
                last_time = current_time
    
                # Step 4: Detect sharp turn and cap velocity
                if abs(error) > 0.6:  # Threshold for sharp turn detection
                    linear_velocity = 2.0  # Reduce speed for sharp turns
                    kd_line = 0.76
                else:
                    linear_velocity = 5.0  # Restore normal speed
                    kd_line = 0.76
    
                if dt > 0:
                    integral_line += error * dt
                    integral_line = max(min(integral_line, max_integral), -max_integral)  # Clamp integral term
    
                derivative_line = (error - last_error) / dt if dt > 0 else 0
                correction = Kp_line * error + Ki_line * integral_line + Kd_line * derivative_line
                last_error = error
                
                # Step 5: Convert linear velocity to angular velocities
                target_angular_velocity = linear_velocity / wheel_radius  # [rad/s]
                left_angular_velocity = target_angular_velocity - (correction * track_width / 2)
                right_angular_velocity = target_angular_velocity + (correction * track_width / 2)
    
                # Step 6: Update shared variables
                share_adjusted_velocity_A.put(left_angular_velocity)
                share_adjusted_velocity_B.put(right_angular_velocity)
    
                # Store the last velocities
                last_left_velocity = left_angular_velocity
                last_right_velocity = right_angular_velocity
            else:
                # No line detected
                current_time = ticks_ms()
                if ticks_diff(current_time, last_line_time) < no_line_timeout:
                    # Force balanced movement
                    reduced_velocity = 0.5 * linear_velocity  # Safe reduced velocity
                    share_adjusted_velocity_A.put(reduced_velocity)
                    share_adjusted_velocity_B.put(reduced_velocity)
    
                    # Update last velocities to match the reduced value
                    last_left_velocity = reduced_velocity
                    last_right_velocity = reduced_velocity
                else:
                    # Stop the motors after timeout
                    share_adjusted_velocity_A.put(0)
                    share_adjusted_velocity_B.put(0)
    
                    # Reset last velocities to avoid propagating old values
                    last_left_velocity = 0
                    last_right_velocity = 0
        else:
            obstacle_nav(20, 20, 1000)
            obstacle_nav(-20, -20, 6000)
            obstacle_nav(-20, 20, 1200)
            mot_A.set_duty(0)
            mot_B.set_duty(0)
            mot_A.disable()
            mot_B.disable()
            mot_A_control.reset()
            mot_B_control.reset()
        yield 0

def task_bump_handling():
    global bump_flag
    while True:    
        if bump_flag:
            # Reverse
            obstacle_nav(-20, -22, 200)  # Reverse with 50% PWM for 1 second
            # Turn 90 degrees
            obstacle_nav(20, -22, 600)   # Turn right with differential PWM
            # Go straight
            obstacle_nav(20, 22, 1500)  # Go straight with 50% PWM for 1 second
            # Turn -90 degrees
            obstacle_nav(-20, 22, 500)  # Turn left with differential PWM
            # Go straight
            obstacle_nav(20, 22, 2500)  # Go straight with 50% PWM for 1 second
            # Turn -90 degrees
            obstacle_nav(-20, 22, 500)  # Turn left with differential PWM
            # Go straight
            obstacle_nav(20, 22, 1350)  # Go straight with 50% PWM for 1 second
            # Turn 90 degrees to return to original direction
            obstacle_nav(20, -22, 500)  # Turn right with differential PWM
            obstacle_nav(20, 20, 10)

            # Ensure motors and shared variables are reset
            share_adjusted_velocity_A.put(0)
            share_adjusted_velocity_B.put(0)
            mot_A.set_duty(0)
            mot_B.set_duty(0)
            mot_A_control.reset()  # Reset feedback controller for Motor A
            mot_B_control.reset()  # Reset feedback controller for Motor B
            # Reset bump flag
            bump_flag = False
        yield 0
    
# Generator Function to control Motor A and log encoder data
def task_motor_A():
    '''!@brief Controls Motor A using adjusted velocity from yaw rate feedback.
        @details This task reads the adjusted velocity for Motor A and applies
                 closed-loop control.
    '''
    mot_A.enable()
    last_time = ticks_ms()
    try:
        while True:
            if bump_flag:
                mot_B.set_duty(0)  # Ensure motor stops during bump handling
                share_adjusted_velocity_B.put(0)
                yield 0
                continue
            current_time = ticks_ms()
            delta_time = ticks_diff(current_time, last_time) / 1000
            last_time = current_time
            enc_A.update()
            position_A = enc_A.get_position_radians() 

            if delta_time == 0:
                delta_time = 0.001

            actual_velocity_A = (enc_A.get_delta() * 6.28) / (1440 * delta_time)
            adjusted_velocity_A = share_adjusted_velocity_A.get()
            PWM_A = mot_A_control.feedback(adjusted_velocity_A, actual_velocity_A)
            mot_A.set_duty(PWM_A)
            
            # Update shared variables
            share_position_A.put(position_A)
            share_velocity_A.put(actual_velocity_A)
            
            yield 0
    finally:
        mot_A.set_duty(0)
        mot_A.disable()

# Generator Function to control Motor B and log encoder data
def task_motor_B():
    '''!@brief   Controls Motor B using adjusted velocity from yaw rate feedback.
        @details This task reads the adjusted velocity for Motor B and applies '
                 closed-loop control.
    '''
    mot_B.enable()
    last_time = ticks_ms()
    try:
        while True:
            current_time = ticks_ms()
            delta_time = ticks_diff(current_time, last_time) / 1000
            last_time = current_time
            if bump_flag:
                mot_B.set_duty(0)  # Ensure motor stops during bump handling
                share_adjusted_velocity_B.put(0)
                yield 0
                continue
            
            enc_B.update()
            position_B = enc_B.get_position_radians()

            if delta_time == 0:
                delta_time = 0.001

            actual_velocity_B = (enc_B.get_delta() * 6.28) / (1440 * delta_time)
            adjusted_velocity_B = share_adjusted_velocity_B.get()
            PWM_B = mot_B_control.feedback(adjusted_velocity_B, actual_velocity_B)
            mot_B.set_duty(PWM_B)
            
            # Update shared variables
            share_position_B.put(position_B)
            share_velocity_B.put(actual_velocity_B)
            
            yield 0
    finally:
        mot_B.set_duty(0)
        mot_B.disable()

# Generator Function to read IMU Data
def task_read_IMU():
    '''!@brief   Reads data from the IMU, including yaw rate and heading, 
                 and computes yaw rate feedback.
        @details This task reads IMU data (yaw rate and heading) and computes
                 a correction for yaw rate using feedback control.
    '''
    imu = IMU_driver.BNO055()
    imu.set_mode(0x0C)  # Set IMU to operation mode (e.g., IMU mode)

    while True:
        # Read IMU data
        yaw_rate = imu.read_yaw()  # rad/s
        heading = imu.read_heading()
        share_yaw.put(yaw_rate)  # Share current yaw rate
        share_heading.put(heading)  # Share current heading

        yield 0

# Generator Function to print data
# Data formatting code taken from ChatGPT
def task_printing():
    '''!@brief   Prints motor velocities and total linear velocity in PuTTY.
        @details This task reads motor velocities from shared variables and calculates
                 the total linear velocity based on wheel velocities and dimensions.
    '''
    start_time = ticks_ms()
    print("Time(s) | Motor A Vel (rad/s) | Motor B Vel (rad/s) | Total Linear Vel (in/s)")
    print("-" * 70)
    
    while True:
        elapsed_time = ticks_diff(ticks_ms(), start_time) / 1000

        # Retrieve shared values
        velocity_A = share_velocity_A.get()
        velocity_B = share_velocity_B.get()

        # Calculate the total linear velocity (average of both wheels)
        total_linear_velocity = (velocity_A + velocity_B) * wheel_radius / 2  # [in/s]

        # Print formatted data
        print(f"{elapsed_time:7.2f} | {velocity_A:17.4f} | {velocity_B:17.4f} | {total_linear_velocity:23.4f}")
 
        yield 0


# Main Program to be Executed
if __name__ == "__main__":
    # Start scheduler
    try:
        ExtInt_A = ExtInt(Pin.cpu.C6, ExtInt.IRQ_RISING, Pin.PULL_DOWN, handle_bump)
        ExtInt_B = ExtInt(Pin.cpu.C8, ExtInt.IRQ_RISING, Pin.PULL_DOWN, handle_bump)
        ExtInt_C = ExtInt(Pin.cpu.C9, ExtInt.IRQ_RISING, Pin.PULL_DOWN, handle_bump)
        
        # Create tasks
        task1 = cotask.Task(task_motor_A, "Task 1", period=17.5, priority=1)
        task2 = cotask.Task(task_motor_B, "Task 2", period=17.5, priority=1)
        task3 = cotask.Task(task_line_following, "Task 3", period=30.0, priority=1)
        task4 = cotask.Task(task_read_IMU, "Task 4", period = 20.0, priority=2)
        task5 = cotask.Task(task_printing, "Task 5", period=250.0, priority=2)
        task6 = cotask.Task(task_bump_handling, "Task 6", period=10.0, priority=1)

        # Append tasks to task list
        cotask.task_list.append(task1)
        cotask.task_list.append(task2)
        cotask.task_list.append(task3)
        cotask.task_list.append(task4)
        cotask.task_list.append(task5)
        cotask.task_list.append(task6)
        while True:
            cotask.task_list.pri_sched()  # Continuously run the scheduler
    except KeyboardInterrupt:
        pass
    finally:
        mot_A.set_duty(0)
        mot_B.set_duty(0)
        mot_A.disable()
        mot_B.disable()
        mot_A_control.reset()
        mot_B_control.reset()