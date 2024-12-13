from pyb import I2C
import struct

class BNO055:
    '''!@brief    A driver class for the BNO055 IMU.
        @details  Objects of this class can be used to retrieve and write
                  information directly from and to the IMU
    '''    
    def __init__(self):   
        '''!@brief    Initializes an I2C object preconfigured in controller
                      mode
            @details  This method configures an I2C object as a controller
                      and sets the I2C register address for future uses
        '''
        self.i2c = I2C(1, I2C.CONTROLLER, baudrate = 400000)
        self.BNO055_ADDR = 0x28
        
    def set_mode(self, mode):
        '''!@brief    Set the operating mode for the IMU
            @details  This method sets the operating mode for the IMU, which
                      can be one of various "fusion" modes that the 
                      BNO055 is capable of.
            @param    mode  Register address of chosen operating mode
        '''
        self.i2c.mem_write(mode, self.BNO055_ADDR, 0x3D)
        
    def read_calibration_status(self):
        '''!@brief    Reads and returns calibration status of IMU sensors
            @details  This method reads the calibration status byte of
                      a given sensor
            @return   A tuple with calibration statuses for system,
                      gyro, accel, and mag.
        '''
        calib_status = self.i2c.mem_read(1, self.BNO055_ADDR, 0x35)[0]
        sys_calib = (calib_status >> 6) & 0x03
        gyro_calib = (calib_status >> 4) & 0x03
        accel_calib = (calib_status >> 2) & 0x03
        mag_calib = calib_status & 0x03
        return sys_calib, gyro_calib, accel_calib, mag_calib
        
    def read_calibration_coefficients(self):
        '''!@brief    Retrieves Calibration Coefficients of the IMU
            @return   Calibration Data as a ByteArray
        '''
        return self.i2c.mem_read(22, self.BNO055_ADDR, 0x55)
    
    def write_calibration_coefficients(self, calib_coeff):
        '''!@brief    Writes Calibration Coefficients of the IMU
            @param    calib_coeff   A ByteArray with Calibration Data
        '''
        self.i2c.mem_write(calib_coeff, self.BNO055_ADDR, 0x55)
        
    def read_euler_angles(self):
        '''!@brief    Reads Euler angles (heading, roll, pitch) from the IMU.
            @return   A tuple containing the heading, roll, and pitch in degrees.
        '''
        euler_ang = self.i2c.mem_read(6, self.BNO055_ADDR, 0x1A)
        heading, roll, pitch = struct.unpack('<hhh', euler_ang)
        return heading/16.0, roll/16.0, pitch/16.0
        
    def read_heading(self):
        '''!@brief Reads only the heading angle from the IMU.
            @return The heading in degrees.
        '''
        heading_byte = self.i2c.mem_read(2, self.BNO055_ADDR, 0x1A)
        (heading,) = struct.unpack('<h', heading_byte)
        return heading/16.0
        
    def read_angular_velocity(self):
        '''!@brief Reads angular velocity values from the IMU.
            @return A tuple containing the angular velocities in the x, y, 
                    and z axes in radians per second.
        '''
        angular_velo = self.i2c.mem_read(6, self.BNO055_ADDR, 0x14)
        omega_x, omega_y, omega_z = struct.unpack('<hhh', angular_velo)
        return omega_x/900.0, omega_y/900.0, omega_z/900.0
    
    def read_yaw(self):
        '''!@brief Reads only the yaw rate from the IMU.
            @return The yaw rate in radians per second.
        '''
        yaw_rate = self.i2c.mem_read(2, self.BNO055_ADDR, 0x18)
        (omega_z,) = struct.unpack('<h', yaw_rate)
        return omega_z/900.0