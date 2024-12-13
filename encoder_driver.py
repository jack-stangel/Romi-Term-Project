from pyb import Timer
AR = 65535
PS = 0
class Encoder:
    '''!@brief    Interface with quadrature encoders
        @details  Objects of this class can be used to record the output of a quadrature
                  encoder attached to a DC motor.
    '''
    def __init__(self, ENC_tim, CH_A_PIN, CH_B_PIN):
        '''!@brief    Initializes objects associated with a quadrature motor encoder.
            @details  This method configures the encoder channels for each motor. It
                      then sets up the timer, and variables to be referenced later.
            @param    ENC_tim   A timer to be configured as the encoder
                      CH_A_PIN  The channel 1 pin associated with the timer chosen for ENC_tim
                      CH_B_PIN  The channel 2 pin associated with the timer chosen for ENC_tim
        '''
        self.EN1 = ENC_tim.channel(1, pin=CH_A_PIN, mode=Timer.ENC_AB)
        self.EN2 = ENC_tim.channel(2, pin=CH_B_PIN, mode=Timer.ENC_AB)
        self.counter = ENC_tim.counter
        self.oldcount = 0
        self.position = 0

    def update(self):
        '''!@brief    Updates encoder position and delta
            @details  This method is called on periodically to track the change
                      in the encoder's position, and record it. If the counter
                      resets, this method accounts for that and only records the
                      true change.
        '''
        self.newcount = -1 * self.counter()
        self.delta = self.newcount - self.oldcount
        self.oldcount = self.newcount
        if self.delta > (AR + 1) / 2:
            self.delta -= AR + 1
        elif self.delta < -(AR + 1) / 2:
            self.delta += AR + 1
        self.position += self.delta

    def get_delta(self):
        '''!@brief    Gets the most recent encoder delta
            @details  
            @return   self.delta  The change in encoder position since the last
                                  check, intended for use in velocity calculations
        '''
        return self.delta

    def get_position_radians(self):
        '''!@brief    Gets the most recent encoder position in radians
            @details
            @return   self.position  The cumulative sum of each change in the
                                     encoder's position.
        '''
        return (self.position * 6.28) / 1440

    def zero(self):
        '''!@brief    Resets the encoder position to zero
            @details
        '''
        self.position = 0