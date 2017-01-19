package org.usfirst.frc.team5689.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team5689.OI;
import org.usfirst.frc.team5689.Robot2017;
import org.usfirst.frc.team5689.RobotMap;

public class DriveSubsystem extends Subsystem {

    private static DriveSubsystem instance;

    public static DriveSubsystem getInstance() {
        if (instance == null) instance = new DriveSubsystem();
        return instance;
    }

    VictorSP leftMotor = new VictorSP(RobotMap.pwmLeftMotor);
    VictorSP rightMotor = new VictorSP(RobotMap.pwmRightMotor);
    Encoder driveEncoder = new Encoder(RobotMap.encoderA, RobotMap.encoderB);
    AHRS driveNav = new AHRS(SerialPort.Port.kUSB);

    RobotDrive robotDrive = new RobotDrive(leftMotor, rightMotor);

    private DriveSubsystem() {
        driveEncoder.setDistancePerPulse(6*Math.PI/360);
    }

    protected void initDefaultCommand() {}

    public void teleopDrive() {
        if (Robot2017.instance.isTest()) {
            robotDrive.tankDrive(-OI.driveController.getY(GenericHID.Hand.kLeft), -OI.driveController.getY(GenericHID.Hand.kRight));
        } else {
            robotDrive.arcadeDrive(-OI.driveController.getY(GenericHID.Hand.kLeft), -OI.driveController.getX(GenericHID.Hand.kRight));
        }
    }

    public void driveForward(double distance) {
        driveForward(distance, false);
    }

    public boolean driveForward(double distance, boolean stopOnCollision) {
        //TODO: More intelligence
        while (driveEncoder.get() < )
        robotDrive.arcadeDrive(0.75, driveNav.getAngle() / 20);
        if (Math.abs(driveNav.getWorldLinearAccelY()) > RobotMap.maxGOnCollision) {
            return false;
        }
        return true;
    }

    public void driveBackward(double distance) {
        driveForward(-distance);
    }

    public void turn(double degrees) {
        //TODO
    }

    public void driveArc(double distance, double endDegrees) {
        //TODO
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void resetNav() {
        driveNav.reset();
    }
}
