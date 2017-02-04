package org.usfirst.frc.team5689;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team5689.DriveRunnable.Status.*;

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

        ckEncoder = new Encoder(RobotMap.encoderA, RobotMap.encoderB);
        ckEncoder.setDistancePerPulse(RobotMap.encoderDistance);
        ckEncoder.setReverseDirection(true);
        ckNavX = new AHRS(RobotMap.portNavx);

        ckDrive = new RobotDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
        ckDrive.setSafetyEnabled(false);
    }


    public void teleDrive(double forward, double rotate) {
        ckDrive.arcadeDrive(forward, rotate);
    }


    public void resetSensors() {
        ckNavX.reset();
        ckEncoder.reset();
    }

    public DriveRunnable driveForward(double distance) {
        return driveForward(distance, false);
    }

    public DriveRunnable driveForward(final double distance, final boolean collisionDetection) {
        return new DriveRunnable() {
            float maxY;

            private void readNav() {
                if (ckNavX.getWorldLinearAccelY() < maxY) {
                    maxY = ckNavX.getWorldLinearAccelY();
                }
            }

            public void run() {
                setStatus(Status.RUNNING);
                resetSensors();
                boolean collision = false;
                while (ckEncoder.getDistance() < (distance - RobotMap.forwardSlowDistance) && !isCancelled() && !collision) {
                    ckDrive.arcadeDrive(1, ckNavX.getAngle() * RobotMap.gyroCorrection);
                    if (collisionDetection) {
                        readNav();
                        if (maxY < -RobotMap.maxCollisionG) collision = true;
                    }
                }
                System.out.println(maxY);
                ckDrive.stopMotor();
                Timer.delay(0.5);
                while (ckEncoder.getDistance() < distance && !isCancelled() && !collision) {
                    ckDrive.arcadeDrive(RobotMap.slowSpeed, ckNavX.getAngle() * RobotMap.gyroCorrection);
                    if (collisionDetection) {
                        readNav();
                        if (maxY < -RobotMap.maxCollisionG) collision = true;
                    }
                }
                System.out.println(maxY);
                ckDrive.stopMotor();
                setStatus(isCancelled() ? CANCELLED : collision ? DEAD : FINISHED);
            }
        };
    }


    public DriveRunnable driveForwardCheckCollision(double distance) {
        return driveForward(distance, true);
    }

    public int turn(double degrees) {
        if (firstLoop) {
            resetSensors();
            firstLoop = false;
        }

        double anglePower = ckNavX.getAngle() - degrees;
        SmartDashboard.putNumber("AnglePower", anglePower);
        anglePower = anglePower * RobotMap.gyroCorrection;
        SmartDashboard.putNumber("MotorPower", anglePower);
        ckDrive.arcadeDrive(0, anglePower);

        //TODO fix this so it ensures we are locked on the angle and then return good
        return RobotMap.notComplete;
    }

}
