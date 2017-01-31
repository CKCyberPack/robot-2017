package org.usfirst.frc.team5689;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.VictorSP;

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
    boolean firstLoop = false;
    boolean collisionDetected = false;


    public DriveTrain() {
        leftFrontMotor = new VictorSP(RobotMap.pwmLeftFrontDrive);
        leftBackMotor = new VictorSP(RobotMap.pwmRightBackDrive);
        rightFrontMotor = new VictorSP(RobotMap.pwmLeftFrontDrive);
        rightBackMotor = new VictorSP(RobotMap.pwmRightBackDrive);

        ckEncoder = new Encoder(RobotMap.encoderA,RobotMap.encoderB);
        ckEncoder.setDistancePerPulse(RobotMap.encoderDistance);
        ckNavX = new AHRS(RobotMap.gyro);


        ckDrive = new RobotDrive(leftBackMotor,leftFrontMotor,rightFrontMotor, rightBackMotor);
       // ckDrive.setInvertedMotor();
    }


    public void teleDrive(double forward, double rotate ) {
        ckDrive.arcadeDrive(forward, rotate);
    }


    public void resetSensors () {
        ckNavX.reset();
        ckEncoder.reset();
        firstLoop = true;
    }


    public int driveForward(double distance){
        if (firstLoop = true) {
            resetSensors();
            firstLoop = false;
        }

        if (ckEncoder.getDistance() >= distance){
            ckDrive.stopMotor();
            return RobotMap.completeGood;
        }
        else {
            ckDrive.arcadeDrive(RobotMap.forwardSpeed, ckNavX.getAngle() / RobotMap.gyroCorrection);
        }
        return RobotMap.notComplete;
    }


    public int driveForwardCheckCollision(double distance){
        if (firstLoop = true){
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

}
