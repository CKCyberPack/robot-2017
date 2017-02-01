package org.usfirst.frc.team5689;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Austin on 2017-01-30.
 */
public class DriveTrain {
    VictorSP leftFrontMotor;
    VictorSP rightFrontMotor;
    VictorSP leftBackMotor;
    VictorSP rightBackMotor;
    RobotDrive ckDrive;
    Encoder ckEncoder;
    AHRS ckNavX;
    boolean firstLoop;
    boolean collisionDetected;


    public DriveTrain() {
        leftFrontMotor = new VictorSP(RobotMap.pwmLeftFrontDrive);
        leftBackMotor = new VictorSP(RobotMap.pwmLeftBackDrive);
        rightFrontMotor = new VictorSP(RobotMap.pwmRightFrontDrive);
        rightBackMotor = new VictorSP(RobotMap.pwmRightBackDrive);

        ckEncoder = new Encoder(RobotMap.encoderA,RobotMap.encoderB);
        ckEncoder.setDistancePerPulse(RobotMap.encoderDistance);
        ckEncoder.setReverseDirection(true);
        ckNavX = new AHRS(RobotMap.portNavx);

        ckDrive = new RobotDrive(leftBackMotor,leftFrontMotor,rightFrontMotor, rightBackMotor);
        ckDrive.setSafetyEnabled(false);
    }


    public void teleDrive(double forward, double rotate ) {
        ckDrive.arcadeDrive(forward, rotate);
    }


    public void resetSensors () {
        ckNavX.reset();
        ckEncoder.reset();
    }


    public int driveForward(double distance){
        if (firstLoop == true){
            resetSensors();
            firstLoop = false;
        }

        if (ckEncoder.getDistance() >= distance){
            ckDrive.stopMotor();
            return RobotMap.completeGood;
        }
        else {
            ckDrive.arcadeDrive(RobotMap.forwardSpeed, ckNavX.getAngle() * RobotMap.gyroCorrection);
        }
        return RobotMap.notComplete;
    }


    public int driveForwardCheckCollision(double distance){
        if (firstLoop == true){
            collisionDetected = false;
        }
        if (ckNavX.getWorldLinearAccelY() >= RobotMap.maxCollisionG){
            collisionDetected = true;
        }

        if (!collisionDetected ){
            return driveForward(distance);
        }
        else{
            return RobotMap.completeBad;
        }

    }

    public int turn(double degrees){
        if (firstLoop == true){
            resetSensors();
            firstLoop = false;
        }

        double anglePower = ckNavX.getAngle() - degrees;
        SmartDashboard.putNumber("AnglePower", anglePower);
        anglePower = anglePower * RobotMap.gyroCorrection;
        SmartDashboard.putNumber("MotorPower", anglePower);
        ckDrive.arcadeDrive(0,anglePower);

        //TODO fix this so it ensures we are locked on the angle and then return good
        return RobotMap.notComplete;
    }

}
