package org.usfirst.frc.team5689;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.usfirst.frc.team5689.DriveRunnable.Status.*;

public class Robot2017 extends IterativeRobot {
    private final static String defaultAuto = "default";
    private final static String leftGear = "left_gear";
    private final static String rightGear = "right_gear";
    private final static String centerGear = "center_gear";
    //Autonomous Selector
    private String autoSelected;
    private SendableChooser<String> chooser;
    //Components
    private XboxController ckController;
    private DriveTrain ckDriveTrain;
    private RopeLift ckRopeLift;
    private GearArm ckGearArm;
    private LED ckLED;
    private PowerDistributionPanel ckPDP;
    private GripPipeline pipeline;
    private List<Rect> points = new ArrayList<>();
    //Variables
    private DriveRunnable runningThread = null;
    private boolean overrideSafety = false;
    private boolean startPressed = false;
    private boolean imgProcReq = false;

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

        pipeline = new GripPipeline();

        new DaemonThread(() ->
        {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(RobotMap.cameraWidth, RobotMap.cameraHeight);

            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("GRIP_Output", RobotMap.cameraWidth, RobotMap.cameraHeight);

            Mat source = new Mat();
            Mat output = new Mat();

            List<MatOfPoint> tPoints;

            while (!Thread.interrupted()) {
                cvSink.grabFrame(source);
                if (imgProcReq) {
                    pipeline.process(source);
                    tPoints = pipeline.findContoursOutput();
                    tPoints.forEach(p -> points.add(Imgproc.boundingRect(p)));
                    imgProcReq = false;
                }
                output = source;
                Mat finalOutput = output;
                points.forEach(p -> Imgproc.rectangle(finalOutput, new Point(p.x, p.y), new Point(p.x + p.width, p.y + p.height), new Scalar(255, 0, 255), 4));
                outputStream.putFrame(finalOutput);
            }
        }).start();
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
        try {
            DriveRunnable r;
            switch (autoSelected) {
                case leftGear:
                    r = ckDriveTrain.drive(84);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.turn(30);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.drive(-36);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.turn(-30);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.drive(204);
                    r.run();
                    while (isAutonomous()) Thread.sleep(100);
                    break;
                case rightGear:
                    r = ckDriveTrain.drive(84);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.turn(-30);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.drive(-36);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.turn(30);
                    r.run();
                    ckDriveTrain.resetSensors();
                    r = ckDriveTrain.drive(204);
                    r.run();
                    while (isAutonomous()) Thread.sleep(100);
                    break;
                case defaultAuto:
                default:
                    r = ckDriveTrain.drive(240);
                    r.run();
                    while (isAutonomous()) Thread.sleep(100);
                    break;
            }

        } catch (Exception e) {
            System.err.println("Error in auto: " + e.getMessage());
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

    private class DaemonThread extends Thread {
        public DaemonThread(Runnable r) {
            super(r);
            setDaemon(true);
        }
    }
}
