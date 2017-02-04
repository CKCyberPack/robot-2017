package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * All "Magic Numbers" Go here
 */
public abstract class RobotMap {

    //Global
    public static int completeGood = 1;
    public static int completeBad = -1;
    public static int notComplete = 0;

    //PWM outputs
    public static int pwmLeftFrontDrive = 1;
    public static int pwmRightFrontDrive = 2;
    public static int pwmLeftBackDrive = 0;
    public static int pwmRightBackDrive = 3;
    public static int pwmRopeMotor = 4;

    //Digital Inputs
    public static int encoderA = 0;
    public static int encoderB = 1;
    public static SerialPort.Port portNavx = SerialPort.Port.kUSB;


    //Pneumatic Controller Module
    public static int pcmPiston = 0;
    public static int pcmLEDCamera = 1;
    public static int pcmLEDRed = 2;
    public static int pcmLEDBlue = 3;
    public static int pcmCompressor = 4;

    //Robot Dependent Variables
    public static float maxCollisionG = 2;
    public static double slowSpeed = 0.4;
    public static double encoderDistance = 6 * Math.PI / 360;
    public static double gyroCorrection = 0.05; //Amount to multiply gyro angle by
    public static double forwardSlowDistance = 60;
    public static double encoderStopSpeed = 1;
}
