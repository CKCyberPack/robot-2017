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
        ckEncoder.setMinRate(RobotMap.encoderStopSpeed);
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

    public DriveRunnable drive(double distance) {
        return drive(distance, false);
    }

    public DriveRunnable drive(final double distance, final boolean collisionDetection) {
        return new DriveRunnable() {
            float maxY;

            private void readNav() {
                if (ckNavX.getWorldLinearAccelY() < maxY) {
                    maxY = ckNavX.getWorldLinearAccelY();
                }
            }

            public void run() {
                setStatus(Status.RUNNING);
                double endLoc = ckEncoder.getDistance() + distance;
                boolean collision = false;
                while (ckEncoder.getDistance() < (endLoc - RobotMap.forwardSlowDistance) && !isCancelled() && !collision) {
                    double turnAmount = ckNavX.getAngle() * RobotMap.gyroStraightKp;
                    ckDrive.arcadeDrive(1, turnAmount);
                    if (collisionDetection) {
                        readNav();
                        if (maxY < -RobotMap.maxCollisionG) collision = true;
                    }
                    Timer.delay(0.05);
                    SmartDashboard.putNumber("Gyro", ckNavX.getAngle());
                    SmartDashboard.putNumber("Turn Amount", turnAmount);
                }

                while (!ckEncoder.getStopped()) {
                    ckDrive.stopMotor();
                }
                while (ckEncoder.getDistance() < endLoc && !isCancelled() && !collision) {
                    double turnAmount = ckNavX.getAngle() * RobotMap.gyroStraightKp;
                    ckDrive.arcadeDrive(RobotMap.slowSpeed, turnAmount);
                    if (collisionDetection) {
                        readNav();
                        if (maxY < -RobotMap.maxCollisionG) collision = true;
                    }
                    Timer.delay(0.05);
                    SmartDashboard.putNumber("Gyro", ckNavX.getAngle());
                    SmartDashboard.putNumber("Turn Amount", turnAmount);
                }

                ckDrive.stopMotor();
                setStatus(isCancelled() ? CANCELLED : collision ? DEAD : FINISHED);
            }
        };
    }


    public DriveRunnable driveForwardCheckCollision(double distance) {
        return drive(distance, true);
    }

    public DriveRunnable turn(double targetAngle) {
        return new DriveRunnable() {
            public void run() {
                setStatus(Status.RUNNING);
                boolean reached = false;
                long reachedTime = 0;
                boolean done = false;
                double integral = 0;
                double previousError = 0;
                while (!done && !isCancelled()) {
                    double error = targetAngle - ckNavX.getAngle();

                    if (RobotMap.gyroTurnKi != 0) {
                        double potentialIGain = (integral + error) * RobotMap.gyroTurnKi;
                        if (potentialIGain < RobotMap.gyroTurnMax) {
                            if (potentialIGain > RobotMap.gyroTurnMin) {
                                integral += error;
                            } else {
                                integral = RobotMap.gyroTurnMin;
                            }
                        } else {
                            integral = RobotMap.gyroTurnMax;
                        }
                    }

                    double derivative = error - previousError;
                    previousError = error;

                    SmartDashboard.putNumber("P", error);
                    SmartDashboard.putNumber("I", integral);
                    SmartDashboard.putNumber("D", derivative);
                    double turnAmount = RobotMap.gyroTurnKp * error + RobotMap.gyroTurnKi * integral + RobotMap.gyroTurnKd * derivative;

                    if (turnAmount > RobotMap.gyroTurnMax) {
                        turnAmount = RobotMap.gyroTurnMax;
                    } else if (turnAmount < RobotMap.gyroTurnMin) {
                        turnAmount = RobotMap.gyroTurnMin;
                    }
                    SmartDashboard.putNumber("Gyro", ckNavX.getAngle());
                    SmartDashboard.putNumber("Turn Amount", turnAmount);

                    ckDrive.arcadeDrive(RobotMap.gyroStraightSpeed, -turnAmount);
                    if (!reached) {
                        if (Math.abs(error) < RobotMap.gyroTolerance) {
                            reached = true;
                            reachedTime = System.currentTimeMillis();
                        }
                    } else {
                        if (Math.abs(ckNavX.getAngle() - targetAngle) < RobotMap.gyroTolerance) {
                            if (System.currentTimeMillis() - reachedTime > 100) {
                                done = true;
                            }
                        } else {
                            reached = false;
                        }
                    }
                    Timer.delay(0.05);
                }
                ckDrive.stopMotor();
                setStatus(isCancelled() ? CANCELLED : FINISHED);
            }
        };
    }

}
