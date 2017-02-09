package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team5689.DriveRunnable.Status.*;

public class Robot2017 extends IterativeRobot {
    final String defaultAuto = "default";
    final String leftGear = "left_gear";
    final String rightGear = "right_gear";
    String autoSelected;
    SendableChooser<String> chooser = new SendableChooser<>();
    XboxController ckController;
    DriveTrain ckDriveTrain;
    private DriveRunnable runningThread = null;
    private boolean yPress;


    @Override
    public void robotInit() {
        chooser.addDefault("Drive Straight", defaultAuto);
        chooser.addObject("Left Gear", leftGear);
        chooser.addObject("Right Gear", rightGear);
        SmartDashboard.putData("Auto choices", chooser);


        ckController = new XboxController(0);
        ckDriveTrain = new DriveTrain();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void teleopPeriodic() {


        SmartDashboard.putNumber("Distance", ckDriveTrain.ckEncoder.getDistance());
        SmartDashboard.putNumber("AccelX", ckDriveTrain.ckNavX.getWorldLinearAccelX());
        SmartDashboard.putNumber("AccelY", ckDriveTrain.ckNavX.getWorldLinearAccelY());

        if (runningThread != null) {
            if (runningThread.getStatus() == CANCELLED || runningThread.getStatus() == FINISHED || runningThread.getStatus() == DEAD) {
                runningThread = null;
            }
        }

        if (runningThread == null) {
            if (ckController.getYButton()) {
                ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
                runningThread = ckDriveTrain.turn(90);
                new Thread(runningThread).start();
            } else if (ckController.getXButton()) {
                ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0.5);
                ckDriveTrain.resetSensors();
                runningThread = ckDriveTrain.driveForwardCheckCollision(84);
                new Thread(runningThread).start();
            } else if (ckController.getAButton()){
                ckDriveTrain.resetSensors();
            }
            else {
                ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                ckDriveTrain.teleDrive(-ckController.getY(GenericHID.Hand.kLeft), -ckController.getX(GenericHID.Hand.kRight));
            }
        } else {
            if (ckController.getBButton()) {
                runningThread.cancel();
            }
        }

    }

    @Override
    public void autonomousInit() {
        autoSelected = chooser.getSelected();
        // autoSelected = SmartDashboard.getString("Auto Selector",
        // defaultAuto);
        System.out.println("Auto selected: " + autoSelected);
    }

    @Override
    public void autonomousPeriodic() {

        switch (autoSelected) {
            case leftGear:
                ckDriveTrain.drive(84);
                ckDriveTrain.resetSensors();
                ckDriveTrain.turn(30);
                ckDriveTrain.resetSensors();
                ckDriveTrain.drive(-36);
                ckDriveTrain.resetSensors();
                ckDriveTrain.turn(-30);
                ckDriveTrain.resetSensors();
                ckDriveTrain.drive(204);
                break;
            case rightGear:
                ckDriveTrain.drive(84);
                ckDriveTrain.resetSensors();
                ckDriveTrain.turn(-30);
                ckDriveTrain.resetSensors();
                ckDriveTrain.drive(-36);
                ckDriveTrain.resetSensors();
                ckDriveTrain.turn(30);
                ckDriveTrain.resetSensors();
                ckDriveTrain.drive(204);
                break;
            case defaultAuto:
            default:
                ckDriveTrain.drive(240);
                break;
        }

    }

    @Override
    public void testInit() {
        super.testInit();

        //VictorSP testing = new VictorSP(2);


    }

    @Override
    public void testPeriodic() {
        while (isTest()) {
            super.testPeriodic();
            LiveWindow.run();
            ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.1);
            ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0.1);
            teleopPeriodic();
            Timer.delay(0.005);
        }
        ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }
}
