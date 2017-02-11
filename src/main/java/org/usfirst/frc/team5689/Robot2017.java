package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
    private PowerDistributionPanel ckPDP;

    //Variables
    private DriveRunnable runningThread = null;
    private boolean yPress;


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
        ckPDP = new PowerDistributionPanel();
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {


        ckRopeLift.testLift(ckController.getTriggerAxis(GenericHID.Hand.kRight));
        if (ckController.getBumper(GenericHID.Hand.kRight)) {
            ckGearArm.firePiston();
        }
        if (ckController.getBumper(GenericHID.Hand.kLeft)) {
            ckGearArm.closePiston();
        }

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
            } else if (ckController.getAButton()) {
                ckDriveTrain.resetSensors();
            } else {
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
        LiveWindow.run();

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
