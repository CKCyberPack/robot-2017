package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team5689.DriveRunnable.Status.*;

public class Robot2017 extends IterativeRobot {
    //Autonomous Selector
    private String autoSelected;
    private SendableChooser<String> chooser;
    private final static String defaultAuto = "default";
    private final static String leftGear = "left_gear";
    private final static String rightGear = "right_gear";
    private final static String centerGear = "center_gear";

    //Components
    private XboxController ckController;
    private DriveTrain ckDriveTrain;
    private RopeLift ckRopeLift;
    private GearArm ckGearArm;
    private LED ckLED;
    private PowerDistributionPanel ckPDP;

    //Variables
    private DriveRunnable runningThread = null;
    private boolean overrideSafety = false;
    private boolean startPressed = false;

    @Override
    public void robotInit() {
        //Create SmartDashboard Chooser
        chooser = new SendableChooser<>();
        chooser.addDefault("Drive Straight", defaultAuto);
        chooser.addObject("Left Gear", leftGear);
        chooser.addObject("Right Gear", rightGear);
        chooser.addObject("Center Gear", centerGear);
        SmartDashboard.putData("Auto choices", chooser);

        //Subsystems
        ckController = new XboxController(0);
        ckDriveTrain = new DriveTrain();
        ckRopeLift = new RopeLift();
        ckGearArm = new GearArm();
        ckLED = new LED();
        ckPDP = new PowerDistributionPanel();
        CameraServer.getInstance().startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        //****Set Safety
        if (ckController.getTriggerAxis(GenericHID.Hand.kLeft) > 0) {
            overrideSafety = true;
            ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.75);
        } else {
            overrideSafety = false;
            ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        }

        //****Rope Lift
        if (overrideSafety) {
            ckRopeLift.dangerClimb(ckController.getTriggerAxis(GenericHID.Hand.kRight));
        } else {
            ckRopeLift.safeClimb(ckController.getTriggerAxis(GenericHID.Hand.kRight), ckPDP);
        }
        //Clear Over Current
        if (ckController.getTriggerAxis(GenericHID.Hand.kRight) == 0) {
            ckRopeLift.clearOverCurrent();
        }

        //****Gear Arm
        if (ckController.getBumper(GenericHID.Hand.kRight)) {
            ckGearArm.firePiston();
            ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0.75);
        }
        if (ckController.getBumper(GenericHID.Hand.kLeft)) {
            ckGearArm.closePiston();
            ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }

        //****Vision LED
        if (ckController.getStartButton()) {
            if (!startPressed) {
                ckLED.toggleVision();
                startPressed = true;
            }
        } else {
            startPressed = false;
        }

        //****Drive Train Auto-Drive
        if (runningThread != null) {
            if (runningThread.getStatus() == CANCELLED || runningThread.getStatus() == FINISHED || runningThread.getStatus() == DEAD) {
                runningThread = null;
            }
        }

        if (runningThread == null) {
            if (ckController.getYButton()) {
                runningThread = ckDriveTrain.turn(90);
                new Thread(runningThread).start();
            } else if (ckController.getXButton()) {
                ckDriveTrain.resetSensors();
                runningThread = ckDriveTrain.driveToGear();
                new Thread(runningThread).start();
            } else if (ckController.getAButton()) {
                ckDriveTrain.resetSensors();
            } else {
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
    }

    @Override
    public void testPeriodic() {
        //LiveWindow.run();

        SmartDashboard.putNumber("Distance", ckDriveTrain.ckEncoder.getDistance());
        SmartDashboard.putNumber("AccelX", ckDriveTrain.ckNavX.getWorldLinearAccelX());
        SmartDashboard.putNumber("AccelY", ckDriveTrain.ckNavX.getWorldLinearAccelY());

        SmartDashboard.putNumber("Current: LBack", ckPDP.getCurrent(RobotMap.pdpLeftBackDrive));
        SmartDashboard.putNumber("Current: LFront", ckPDP.getCurrent(RobotMap.pdpLeftFrontDrive));
        SmartDashboard.putNumber("Current: RBack", ckPDP.getCurrent(RobotMap.pdpRightBackDrive));
        SmartDashboard.putNumber("Current: RFront", ckPDP.getCurrent(RobotMap.pdpRightFrontDrive));
        SmartDashboard.putNumber("Current: Rope", ckPDP.getCurrent(RobotMap.pdpRopeMotor));

        teleopPeriodic();
    }
}
