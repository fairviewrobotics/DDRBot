#!/usr/bin/env python3
'''
    This is a demo program showing how to use Mecanum control with the
    RobotDrive class.
'''

import wpilib
from wpilib import RobotDrive

class MyRobot(wpilib.SampleRobot):

    # Channels for the wheels
    frontLeftChannel = 0
    rearLeftChannel = 2 #not victor(talon)
    frontRightChannel = 1
    rearRightChannel = 3

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0

    def robotInit(self):
        '''Robot initialization function'''

        self.rlMotor = wpilib.Victor(2)
        self.rrMotor = wpilib.Talon(3)
        self.frMotor = wpilib.Victor(1)
        self.flMotor = wpilib.Victor(0)
        
        self.rlMotor.isInverted = True
               
        self.flMotor.set(0)
        self.rlMotor.set(0)
        self.frMotor.set(0)
        self.rrMotor.set(0)

        self.motorArr = [self.flMotor, self.rlMotor, self.frMotor, self.rrMotor]

        self.stick = wpilib.Joystick(self.joystickChannel)

        self.maxSpeed = .5#This is the current speed of the robot
        self.totalMaxSpeed = 1#This is the max speed
        self.currSpeed = [0, 0, 0, 0]

    def operatorControl(self):
        '''Runs the motors with Mecanum drive.'''

        while self.isOperatorControl() and self.isEnabled():
            #Forward
            if self.stick.getRawButton(13):
                self.accelTo(self.maxSpeed, 0.0025, self.rearRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.frontRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.frontLeftChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.rearLeftChannel)
            elif self.stick.getRawButton(15):#Backward
                self.accelTo(-self.maxSpeed, 0.0025, self.rearRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.frontRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.frontLeftChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.rearLeftChannel)
            elif self.stick.getRawButton(2):#Turn Left
                self.accelTo(self.maxSpeed, 0.0025, self.rearRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.frontRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.frontLeftChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.rearLeftChannel)
            elif self.stick.getRawButton(3):#Turn Right
                self.accelTo(-self.maxSpeed, 0.0025, self.rearRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.frontRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.frontLeftChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.rearLeftChannel)
            elif self.stick.getRawButton(14):#Strafe right
                self.accelTo(-self.maxSpeed, 0.0025, self.rearRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.frontRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.frontLeftChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.rearLeftChannel)
            elif self.stick.getRawButton(16):#Strafe Left
                self.accelTo(self.maxSpeed, 0.0025, self.rearRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.frontRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, self.frontLeftChannel)
                self.accelTo(-self.maxSpeed, 0.0025, self.rearLeftChannel)
            elif self.stick.getRawButton(10):#Increase Speed
                if not (self.maxSpeed > self.totalMaxSpeed):
                    self.maxSpeed += 0.001
            elif self.stick.getRawButton(8):#Decrease Speed
                if not (self.maxSpeed < 0):
                    self.maxSpeed -= 0.001
            else:#Stop
                #Change 0.0050 to make deceleration slower or fasters
                self.accelTo(0, 0.0050, 0)
                self.accelTo(0, 0.0050, 1)
                self.accelTo(0, 0.0050, 2)
                self.accelTo(0, 0.0050, 3)

            self.moveMotors()

    def accelTo(self, targetSpeed, speed, mNum):
        if self.currSpeed[mNum] < targetSpeed:
            self.currSpeed[mNum] += speed
        elif self.currSpeed[mNum] > targetSpeed:
            self.currSpeed[mNum] -= speed

    def moveMotors(self):
        for x in range(len(self.motorArr)):
            if x < 2:
                self.motorArr[x].set(self.currSpeed[x])
            else:
                self.motorArr[x].set(self.currSpeed[x])

    def goBackwards(self, speed):
        self.goForward(-speed)

    def rotateLeft(self, speed):
        self.rotateRight(-speed)

    def rotateRight(self, speed):
        self.motorArr[0].set(speed)
        self.motorArr[1].set(speed)
        self.motorArr[2].set(speed)
        self.motorArr[3].set(speed)

    def strafe(self, speed):
        self.motorArr[0].set(-speed)
        self.motorArr[1].set(speed)
        self.motorArr[2].set(speed)
        self.motorArr[3].set(-speed)

    def resetMotors(self):
        self.flMotor.set(0)
        self.rlMotor.set(0)
        self.frMotor.set(0)
        self.rrMotor.set(0)


if __name__ == '__main__':
    wpilib.run(MyRobot)
