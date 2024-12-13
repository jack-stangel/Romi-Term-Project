from time import ticks_ms, ticks_diff

class ClosedLoop:
    '''!@brief A driver class for closed-loop control of Romi's Velocity.
        @details Implements a PID controller with saturation limits for integral and PWM terms.
    '''
    def __init__(self, kp, ki, kd, max_integral=100, pwm_min=-100, pwm_max=100):
        '''!@brief Initializes the PID controller with specified gains and limits.
            @param kp Proportional gain.
            @param ki Integral gain.
            @param kd Derivative gain.
            @param max_integral Maximum allowable value for the integral term.
            @param pwm_min Minimum PWM output.
            @param pwm_max Maximum PWM output.
        '''
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.last_time = ticks_ms()
        self.max_integral = max_integral
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max

    def feedback(self, specified_velo, actual_velo):
        '''!@brief Computes the PID output based on specified and actual velocities.
            @param specified_velo Desired velocity for the motor.
            @param actual_velo Current velocity from the encoder.
            @return PWM signal for motor control (clamped within pwm_min and pwm_max).
        '''
        # Calculate error
        error = specified_velo - actual_velo
        current_time = ticks_ms()
        dt = ticks_diff(current_time, self.last_time) / 1000  # Convert ms to seconds
        self.last_time = current_time

        # Compute proportional term
        prop_gain = self.kp * error

        # Compute integral term with saturation
        if dt > 0:
            self.integral += error * dt
            self.integral = max(min(self.integral, self.max_integral), -self.max_integral)  # Clamp integral term
        int_gain = self.ki * self.integral

        # Compute derivative term
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        der_gain = self.kd * derivative
        self.last_error = error

        # Calculate total output and apply PWM limits
        PWM_signal = prop_gain + int_gain + der_gain
        PWM = max(min(PWM_signal, self.pwm_max), self.pwm_min)

        return PWM

    def reset(self):
        '''!@brief Resets the integral term, last error, and timestamp.'''
        self.integral = 0
        self.last_error = 0
        self.last_time = ticks_ms()
