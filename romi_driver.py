from pyb import Pin, Timer

class Romi:
    '''!@brief    A driver class for one Romi motor.
        @details  Objects of this class can be used to apply PWM to a given
                  DC motor on Romi
    '''
    def __init__(self, PWM_tim, EN, DIR, EFF):
        '''!@brief    Initializes and returns an object associated with a DC motor.
            @details  This method configures the PWM channels for each motor. It
                      then configures the enable pin, direction pin, and effort
                      pin.
            @param    PWM_tim  A timer to be configured as the PWM
                      EN       The enable pin associated with the motor being configured
                      DIR      The direction pin associated with the motor's spin
                      EFF      Pin associated with timer chosen for PWM_tim
        '''
        self.IN = PWM_tim.channel(1, mode=Timer.PWM, pin=EFF)
        self.DIR = Pin(DIR, mode=Pin.OUT_PP)
        self.ON = Pin(EN, mode=Pin.OUT_PP)
        self.ON.low()

    def set_duty(self, duty):
        '''!@brief    Set the PWM duty cycle for the DC motor.
            @details  This method sets the duty cycle to be sent
                      to the Romi Motor to a given level. Positive values
                      cause effort in one direction, negative values
                      in the opposite direction.
            @param    duty  A signed number holding the duty
                            cycle of the PWM signal sent to the L6206
        '''
        if duty > 0:
            self.IN.pulse_width_percent(duty)
            self.DIR.low()
        else:
            self.IN.pulse_width_percent(abs(duty))
            self.DIR.high()

    def enable(self):
        '''!@brief    Enable one channel of the Romi Motors.
            @details  This method sets the enable pin associated with one
                      channel of the Romi motors high in order to enable that
                      channel of the motor driver.
        '''
        self.ON.high()

    def disable(self):
        '''!@brief    Disable one channel of the Romi Motors.
            @details  This method clears the enable pin associated with one
                      channel of the Romi Motors high in order to disable that
                      channel of the motor driver.
        '''
        self.ON.low()