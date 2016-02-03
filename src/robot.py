#!/usr/bin/env python3
'''
    This is a demo program showing how to use Mecanum control with the
    RobotDrive class.
'''

import wpilib
from wpilib import RobotDrive

class MyRobot(wpilib.SampleRobot):

    # Channels for the wheels
    frontLeftChannel = 2
    rearLeftChannel = 3
    frontRightChannel = 1
    rearRightChannel = 0

    # The channel on the driver station that the joystick is connected to
    joystickChannel = 0;


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

        self.maxSpeed = .5
        self.currSpeed = [0, 0, 0, 0]

    def operatorControl(self):
        '''Runs the motors with Mecanum drive.'''

        while self.isOperatorControl() and self.isEnabled():
            #Forward
            if self.stick.getRawButton(13):
                self.accelTo(self.maxSpeed, 0.0025, rearRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, frontRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, frontLeftChannel)
                self.accelTo(self.maxSpeed, 0.0025, rearLeftChannel)
            elif self.stick.getRawButton(15):#Backward
                self.accelTo(-self.maxSpeed, 0.0025, rearRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, frontRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, frontLeftChannel)
                self.accelTo(-self.maxSpeed, 0.0025, rearLeftChannel)
            elif self.stick.getRawButton(3):#Turn Left
                self.accelTo(self.maxSpeed, 0.0025, rearRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, frontRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, frontLeftChannel)
                self.accelTo(-self.maxSpeed, 0.0025, rearLeftChannel)
            elif self.stick.getRawButton(2):#Turn Right
                self.accelTo(-self.maxSpeed, 0.0025, rearRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, frontRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, frontLeftChannel)
                self.accelTo(self.maxSpeed, 0.0025, rearLeftChannel)
            elif self.stick.getRawButton(16):#Strafe right
                self.accelTo(self.maxSpeed, 0.0025, rearRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, frontRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, frontLeftChannel)
                self.accelTo(-self.maxSpeed, 0.0025, rearLeftChannel)
            elif self.stick.getRawButton(14):#Strafe Left
                self.accelTo(self.maxSpeed, 0.0025, rearRightChannel)
                self.accelTo(-self.maxSpeed, 0.0025, frontRightChannel)
                self.accelTo(self.maxSpeed, 0.0025, frontLeftChannel)
                self.accelTo(-self.maxSpeed, 0.0025, rearLeftChannel)
            else:#Stop
                self.accelTo(0, 0.0025, 0)
                self.accelTo(0, 0.0025, 1)
                self.accelTo(0, 0.0025, 2)
                self.accelTo(0, 0.0025, 3)

            self.moveMotors()
#             print("0: " + str(self.currSpeed[0]) + ", 1: " + str(self.currSpeed[1]) + ", 2: " + str(self.currSpeed[2]) + ", 3:" + str(self.currSpeed[3]))

#         self.resetMotors()
#
#         while self.isOperatorControl() and self.isEnabled():
#             if self.stick.getRawButton(13):
#                 self.flMotor.set(self.defaultSpeed)
#             elif self.stick.getRawButton(15):
#                 self.rlMotor.set(self.defaultSpeed)
#             elif self.stick.getRawButton(14):
#                 self.frMotor.set(self.defaultSpeed)
#             elif self.stick.getRawButton(16):
#                 self.rrMotor.set(self.defaultSpeed)

#         while self.isOperatorControl() and self.isEnabled():
#
#             # Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
#             # This sample does not use field-oriented drive, so the gyro input is set to zero.
#             if (self.stick.getRawButton(13)):
#                 yAxis = self.defaultSpeed;
#             elif (self.stick.getRawButton(15)):
#                 yAxis = -self.defaultSpeed;
#             else:
#                 yAxis = 0;
#
#             self.goForward(yAxis)
#
#             if self.stick.getRawButton(2):
#                 rotateRightAxis = self.defaultSpeed;
#             elif self.stick.getRawButton(3):
#                 rotateLeftAxis = self.defaultSpeed;
#             else:
#                 rotateRightAxis = 0;
#                 rotateLeftAxis = 0;
#
#             if rotateRightAxis > 0:
#                 self.rotateRight(rotateRightAxis)
#             elif rotateLeftAxis > 0:
#                 self.rotateLeft(rotateLeftAxis)
#
#             if (self.stick.getRawButton(14)):
#                 xAxis = self.defaultSpeed*2;
#             elif (self.stick.getRawButton(16)):
#                 xAxis = -self.defaultSpeed*2;
#             else:
#                 xAxis = 0;
#
#             if(abs(yAxis) <= 0.1 and abs(xAxis) >= 0.1):
#                 self.strafe(xAxis)
#
#             self.goForward(yAxis)
#
#             # if abs(strafeRightAxis) >= 0.1:
#             #   self.strafeRight(strafeRightAxis)
#             wpilib.Timer.delay(0.005)  # wait 5ms to avoid hogging CPU cycles


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
                self.motorArr[x].set(-self.currSpeed[x])

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
