#!/usr/bin/env python3
'''
    This is a demo program showing how to use Mecanum control with the
    RobotDrive class.
'''

import wpilib
from wpilib import RobotDrive

class MyRobot(wpilib.SampleRobot):
    
    # Channels for the wheels
    frontLeftChannel    = 2
    rearLeftChannel     = 3
    frontRightChannel   = 1
    rearRightChannel    = 0
    
    # The channel on the driver station that the joystick is connected to
    joystickChannel     = 0;

    def robotInit(self):
        '''Robot initialization function'''
        
        self.flMotor = wpilib.Talon(2)
        self.rlMotor = wpilib.Talon(3)
        self.frMotor = wpilib.Talon(1)
        self.rrMotor = wpilib.Talon(0)
        
        self.flMotor.set(0)
        self.rlMotor.set(0)
        self.frMotor.set(0)
        self.rrMotor.set(0)
        
        self.motorArr = [self.flMotor, self.rlMotor, self.frMotor, self.rrMotor]

        self.stick = wpilib.Joystick(self.joystickChannel)
    
    def operatorControl(self):
        '''Runs the motors with Mecanum drive.'''
        
        while self.isOperatorControl() and self.isEnabled():
            
            # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
            # This sample does not use field-oriented drive, so the gyro input is set to zero.
            forwardAxis = self.stick.getRawAxis(2) * -1
            backwardAxis = self.stick.getRawAxis(3)
            yAxis = forwardAxis + backwardAxis
            
            rotateRightAxis = self.stick.getRawAxis(0) * -1
            
            strafeRightAxis = self.stick.getRawAxis(4) * -1
            
            self.goForward(yAxis)
            if abs(rotateRightAxis) >= 0.1:
                self.rotateRight(rotateRightAxis)
            if abs(strafeRightAxis) >= 0.1:
                self.strafeRight(strafeRightAxis)
            wpilib.Timer.delay(0.005)   # wait 5ms to avoid hogging CPU cycles
    
    def goForward(self, speed):
        for x in range(len(self.motorArr)):
            if x<2:
                self.motorArr[x].set(speed)
            else:
                self.motorArr[x].set(-speed)          
                  
    def goBackwards(self, speed):
        self.goForward(-speed)
        
    def rotateLeft(self, speed):
        self.motorArr[0].set(speed)
        self.motorArr[1].set(speed)
        self.motorArr[2].set(speed)
        self.motorArr[3].set(speed)
        
    def rotateRight(self, speed):
        self.rotateLeft(-speed)
    
    def strafeLeft(self, speed):
        self.motorArr[0].set(-speed)
        self.motorArr[1].set(speed)
        self.motorArr[2].set(speed)
        self.motorArr[3].set(-speed)
        
    def strafeRight(self, speed):
        self.strafeLeft(-speed)
        
    def resetMotors(self):
        self.flMotor.set(0)
        self.rlMotor.set(0)
        self.frMotor.set(0)
        self.rrMotor.set(0)
    
    
if __name__ == '__main__':
    wpilib.run(MyRobot)

