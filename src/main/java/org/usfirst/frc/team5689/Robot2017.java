package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot2017 extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser<String> chooser = new SendableChooser<>();


    XboxController ckController;
    DriveTrain ckDriveTrain;


    @Override
    public void robotInit() {
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
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

        SmartDashboard.putNumber("Gyro", ckDriveTrain.ckNavX.getAngle());
        SmartDashboard.putNumber("Distance", ckDriveTrain.ckEncoder.getDistance());


        if (ckController.getAButton()){
            ckDriveTrain.turn(90);
            ckController.setRumble(GenericHID.RumbleType.kRightRumble,1);
        }
        else if (ckController.getXButton()){
            ckDriveTrain.driveForward(100);
            ckController.setRumble(GenericHID.RumbleType.kLeftRumble,1);
        }
        else {
            ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
            ckDriveTrain.firstLoop = true;
            ckDriveTrain.teleDrive(-ckController.getY(GenericHID.Hand.kLeft),-ckController.getX(GenericHID.Hand.kRight));
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
            case customAuto:
                // Put custom auto code here
                break;
            case defaultAuto:
            default:
                // Put default auto code here
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
        while (isTest() && isOperatorControl()) {
            LiveWindow.run();
            ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.1);
            ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0.1);
            ckDriveTrain.teleDrive(-ckController.getY(GenericHID.Hand.kLeft), -ckController.getX(GenericHID.Hand.kRight));
            Timer.delay(0.005);
        }
        ckController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        ckController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }
}
