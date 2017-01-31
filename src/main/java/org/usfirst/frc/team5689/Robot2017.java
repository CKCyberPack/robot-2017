package org.usfirst.frc.team5689;

import com.kauailabs.navx.frc.AHRS;
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
    DriveTrain ckDrive;


    @Override
    public void robotInit() {
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);


        ckController = new XboxController(0);
        ckDrive = new DriveTrain();

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


//        if (ckController.getAButton()) {
//            if (!bTriggerPressed) {
//                //First loop
//                ckGyro.reset();
//            }
//
//            if (ckGyro.getAngle() < 80) {
//                ckDrive.arcadeDrive(0, -0.5);
//            } else if (ckGyro.getAngle() < 89) {
//                ckDrive.arcadeDrive(0, -0.33);
//
//            } else if (ckGyro.getAngle() > 100) {
//                ckDrive.arcadeDrive(0, 0.5);
//            } else if (ckGyro.getAngle() > 91) {
//                ckDrive.arcadeDrive(0, 0.33);
//            } else {
//                ckDrive.stopMotor();
//            }
//
//            bTriggerPressed = true;
//        } else if (ckController.getXButton() && !wallCollision) {
        if (ckController.getXButton()){
            ckDrive.driveForward(10);
        }
        else {
            ckDrive.teleDrive(-ckController.getY(GenericHID.Hand.kLeft),-ckController.getX(GenericHID.Hand.kRight));
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
        super.testPeriodic();
        LiveWindow.run();
    }
}
