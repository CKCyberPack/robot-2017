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
    public static float maxCollisionG = 0.5f;
    public static double slowSpeed = 0.4;
    public static double encoderDistance = 6 * Math.PI / 360;
    public static double forwardSlowDistance = 60;
    public static double encoderStopSpeed = 1;
    public static double gyroStraightKp = -0.05; //Negative value to go left
    public static double gyroTurnKp = 0.03; //0.03
    public static double gyroTurnKi = 0.01; //0.007
    public static double gyroTurnKd = 0.175; //0.1
    public static double gyroTurnMax = 0.9;
    public static double gyroTurnMin = -gyroTurnMax;
    public static double gyroTolerance = 1; //Number of degrees off by
}
