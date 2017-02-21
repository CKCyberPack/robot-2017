package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.SerialPort;

/**
 * All "Magic Numbers" Go here
 */
public abstract class RobotMap {

    //PWM outputs
    public static int pwmLeftFrontDrive = 2;
    public static int pwmRightFrontDrive = 3;
    public static int pwmLeftBackDrive = 1;
    public static int pwmRightBackDrive = 4;
    public static int pwmRopeMotor = 5;

    //Digital Inputs
    public static int encoderA = 0;
    public static int encoderB = 1;
    public static SerialPort.Port portNavx = SerialPort.Port.kUSB2;

    //Relay Outputs
    public static int relayRed = 0;
    public static int relayBlue = 1;


    //Pneumatic Controller Module
    public static int pcmPiston = 4;
    public static int pcmLEDCamera = 0;


    //Motor PDP Inputs
    public static int pdpLeftFrontDrive = 0;
    public static int pdpRightFrontDrive = 1;
    public static int pdpLeftBackDrive = 15;
    public static int pdpRightBackDrive = 2;
    public static int pdpRopeMotor = 3;

    //Vision Variables
    public static int cameraWidth = 640 / 2;
    public static int cameraHeight = 480 / 2;
    public static int cameraExposure = 25;

    //Robot Dependent Variables
    public static double ropeOverCurrent = 75;
    public static double driveDeadzone = 0.2;
    public static float  maxCollisionG = 0.5f;
    public static double slowSpeed = 0.4;
    public static double encoderDistance = 6 * Math.PI / 360;
    public static double forwardSlowDistance = 60;
    public static double encoderStopSpeed = 1;
    public static double gyroStraightKp = 0.045;
    public static double gyroStraightSpeed = 0.75;
    public static double gyroTurnKp = 0.03; //0.03
    public static double gyroTurnKi = 0.01; //0.007
    public static double gyroTurnKd = 0.175; //0.1
    public static double gyroTurnMax = 0.9;
    public static double gyroTurnMin = -gyroTurnMax;
    public static double gyroTolerance = 2; //Number of degrees off by
    public static double gyroJitterAngle = 5;
    public static double visionTurn = 0.3;
    public static double visionForward = 0.6;
    public static double visionMaxG = 0.4;
    public static double dumbTurn = 0.5;
    public static double dumbTurnTolerance = 2;
}
