package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot2017 extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser<String> chooser = new SendableChooser<>();

    BuiltInAccelerometer ckAcc;

    VictorSP leftMotor;
    VictorSP rightMotor;
    XboxController ckController;
    RobotDrive ckDrive;
    ADXRS450_Gyro ckGyro;

    boolean bTriggerPressed = false;


    @Override
    public void robotInit() {
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);


        leftMotor = new VictorSP(0);
        rightMotor = new VictorSP(1);
        ckController = new XboxController(0);
        ckDrive = new RobotDrive(leftMotor, rightMotor);
        ckGyro = new ADXRS450_Gyro();
        ckGyro.calibrate();
        ckAcc = new BuiltInAccelerometer();
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

        SmartDashboard.putNumber("Gyro", ckGyro.getAngle());
        SmartDashboard.putNumber("Left Motor", leftMotor.getSpeed());
        SmartDashboard.putNumber("Right Motor", rightMotor.getSpeed());
        SmartDashboard.putNumber("Accelerometer X", ckAcc.getX());
        SmartDashboard.putNumber("Accelerometer Y", ckAcc.getY());
        SmartDashboard.putNumber("Accelerometer Z", ckAcc.getZ());

        if (ckController.getAButton()) {
            if (bTriggerPressed == false) {
                //First loop
                ckGyro.reset();
            }

            if (ckGyro.getAngle() < 80) {
                ckDrive.arcadeDrive(0, -0.5);
            } else if (ckGyro.getAngle() < 89) {
                ckDrive.arcadeDrive(0, -0.33);

            } else if (ckGyro.getAngle() > 100) {
                ckDrive.arcadeDrive(0, 0.5);
            } else if (ckGyro.getAngle() > 91) {
                ckDrive.arcadeDrive(0, 0.33);
            } else {
                ckDrive.stopMotor();
            }

            bTriggerPressed = true;
        } else if (ckController.getXButton()) {
            if (bTriggerPressed == false) {
                //First loop
                ckGyro.reset();
            }
            //TODO - Use the built in accelerometer to sense a collision and stop
            ckDrive.arcadeDrive(0.4, ckGyro.getAngle() / 20);
            bTriggerPressed = true;
        } else {
            ckDrive.tankDrive(-ckController.getY(GenericHID.Hand.kLeft), -ckController.getY(GenericHID.Hand.kRight));
            bTriggerPressed = false;
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

        VictorSP testing = new VictorSP(2);


    }

    @Override
    public void testPeriodic() {
        super.testPeriodic();
    }
}
