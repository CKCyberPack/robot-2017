package org.usfirst.frc.team5689;

//import edu.wpi.cscore.CvSink;
//import edu.wpi.cscore.CvSource;
//import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.opencv.core.*;
//import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.usfirst.frc.team5689.DriveRunnable.Status.*;

public class Robot2017 extends IterativeRobot {
    private final static String centerGear = "center_gear";
    private final static String leftGear = "left_gear";
    private final static String rightGear = "right_gear";
    private final static String doNothing = "do_nothing";
    public static LED ckLED;
    public static boolean imgProcReq = false;
   // public static Rect[] visionBiggest = new Rect[2];
    public static double mw = 0;
    public static boolean left = false;
    DriveTrain drive;
    ADXRS450_Gyro gyro;
    //Autonomous Selector
    private String autoSelected;
    private SendableChooser<String> chooser;
    //Components
    private XboxController ckController;
    private DriveTrain ckDriveTrain;
    private RopeLift ckRopeLift;
    private GearArm ckGearArm;
    private PowerDistributionPanel ckPDP;
   // private GripPipeline pipeline;
  //  private List<Rect> points = new ArrayList<>();
    //Variables
    private DriveRunnable runningThread = null;
    private boolean overrideSafety = false;
    private boolean startPressed = false;
    private boolean ledOn = false;
    private boolean autoDone = false;

    @Override
    public void robotInit() {
        //Create SmartDashboard Chooser
        chooser = new SendableChooser<>();
        chooser.addDefault("Do Nothing", doNothing);
        chooser.addObject("Center Gear", centerGear);
        chooser.addObject("Left Gear", leftGear);
        chooser.addObject("Right Gear", rightGear);
        SmartDashboard.putData("Auto choices", chooser);

        //Subsystems
        ckController = new XboxController(0);
        ckDriveTrain = new DriveTrain();
        ckRopeLift = new RopeLift();
        ckGearArm = new GearArm();
        ckLED = new LED();
        ckPDP = new PowerDistributionPanel();

       // pipeline = new GripPipeline();


        //Camera stuff
//        new DaemonThread(() -> {
//            UsbCamera camera = new UsbCamera("USB Camera", 0);
//            camera.setResolution(RobotMap.cameraWidth, RobotMap.cameraHeight);
//            camera.setExposureManual(RobotMap.cameraExposure);
//            camera.setFPS(30);
//
//            CvSink cvSink = new CvSink("GRIP_Input");
//            cvSink.setSource(camera);
//            CvSource outputStream = CameraServer.getInstance().putVideo("GRIP_Output", RobotMap.cameraWidth,  RobotMap.cameraHeight);
//
//            Mat source = new Mat();
//            Mat output = new Mat();
//
//            List<MatOfPoint> tPoints;
//
//            Timer.delay(1);
//
//            while (!Thread.interrupted()) {
//                cvSink.grabFrame(source);
//                if (source.empty()) {
//                    Timer.delay(0.1);
//                    continue;
//                }
//                long start = System.currentTimeMillis();
//                if (imgProcReq) {
//                    if (isDisabled()) imgProcReq = false;
//                    points.clear();
//                  //  pipeline.process(source);
//                   // tPoints = pipeline.findContoursOutput();
//                  //  tPoints.forEach(p -> points.add(Imgproc.boundingRect(p)));
//                    visionBiggest[0] = null;
//                    visionBiggest[1] = null;
//                    points.forEach(p -> {
//                        if (visionBiggest[0] == null || p.area() > visionBiggest[0].area()) {
//                            visionBiggest[1] = visionBiggest[0];
//                            visionBiggest[0] = p;
//                        } else if (visionBiggest[1] == null || p.area() > visionBiggest[1].area()) {
//                            visionBiggest[1] = p;
//                        }
//                    });
//                    if (visionBiggest[0] != null) {
//                        if (visionBiggest[1] == null) {
//                            visionBiggest[1] = visionBiggest[0];
//                        }
//                        mw = ((visionBiggest[0].width + visionBiggest[1].width) / 2);
//                        mw += visionBiggest[0].x + visionBiggest[1].x;
//                        mw /= 2;
//                        left = visionBiggest[0].x < visionBiggest[1].x;
//                        long end = System.currentTimeMillis();
//                    } else {
//                    }
//                }
//                output = source;
//                if (visionBiggest[1] != null) {
//                    Imgproc.rectangle(output, new Point(visionBiggest[0].x, visionBiggest[0].y), new Point(visionBiggest[0].x + visionBiggest[0].width, visionBiggest[0].y + visionBiggest[0].height), new Scalar(255, 0, 255));
//                    Imgproc.rectangle(output, new Point(visionBiggest[1].x, visionBiggest[1].y), new Point(visionBiggest[1].x + visionBiggest[1].width, visionBiggest[1].y + visionBiggest[1].height), new Scalar(255, 0, 255));
//                }
//                outputStream.putFrame(output);
//                try {
//                    Thread.sleep(30);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//        }).start();
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
        if (runningThread != null) {
            runningThread.cancel();
            runningThread = null;
        }
        imgProcReq = false;
        ckLED.visionOff();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void teleopInit() {
    	if (runningThread != null) {
    		runningThread.cancel();
    		runningThread = null;
    	}
    	ckLED.visionOff();
    	imgProcReq = false;

        if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue){
            ckLED.blueOn();
            ckLED.redOff();
        }else {
            ckLED.redOn();
            ckLED.blueOff();
        }
    }

    @Override
    public void teleopPeriodic() {
        //**** TESTING - COMMENT ME OUT!

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

        if (ckController.getBackButton()){
            imgProcReq = true;
            new DaemonThread(new Runnable() {public void run() {Timer.delay(0.5);imgProcReq = false;}}).start();
        }

        //****LED
        if (ckController.getStickButton(GenericHID.Hand.kRight)){
            if (!ledOn){
                //ckLED.blinkPatternP();
                ckLED.redOn();
                ckLED.blueOn();
                ledOn = true;
            }
        }

        if (ckController.getStickButton(GenericHID.Hand.kLeft)){
            //ckLED.blinkOff();
            ckLED.redOff();
            ckLED.blueOff();
            ledOn = false;
        }

        //****Drive Train Auto-Drive
        if (runningThread != null) {
            if (runningThread.getStatus() == CANCELLED || runningThread.getStatus() == FINISHED || runningThread.getStatus() == DEAD) {
                runningThread = null;
            }
        }

        if (runningThread == null) {
            /*if (ckController.getYButton()) {
                runningThread = ckDriveTrain.turn(90);
                runningThread.start();
            } else if (ckController.getAButton()) {
                ckDriveTrain.resetSensors();
            }*/
            if (ckController.getAButton()) {
                runningThread = ckDriveTrain.driveVision();
                runningThread.start();
            } else if (ckController.getXButton()) {
                ckDriveTrain.resetSensors();
                runningThread = ckDriveTrain.turn(-30);
                runningThread.start();
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
        autoSelected = SmartDashboard.getString("DB/String 0", "do_nothing");
        System.out.println("Auto selected: " + autoSelected);
        autoDone = false;
        if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue){
            ckLED.blueOn();
            ckLED.redOff();
        }else {
            ckLED.redOn();
            ckLED.blueOff();
        }
    }

    @Override
    public void autonomousPeriodic() {

        long end;
        double error;
        switch (autoSelected) {
            case leftGear:
                ckDriveTrain.resetSensors();
                ckLED.visionOn();
                end = System.currentTimeMillis() + 1450;
                while (isAutonomous() && System.currentTimeMillis() < end) {
                    ckDriveTrain.ckDrive.arcadeDrive(RobotMap.visionForward, 0);
                }
                if (!isAutonomous()) break;
                ckDriveTrain.ckDrive.stopMotor();
                dumbTurn(60);
                ckDriveTrain.ckDrive.stopMotor();
                if (!isAutonomous()) break;
                gear();
                ckDriveTrain.ckDrive.stopMotor();
                if (!isAutonomous()) break;
                dumbTurn(0);
                ckDriveTrain.ckDrive.stopMotor();
                if (!isAutonomous()) break;
                end = System.currentTimeMillis() + 3500;
                while (isAutonomous() && System.currentTimeMillis() < end) {
                    ckDriveTrain.ckDrive.arcadeDrive(RobotMap.visionForward, 0);
                }
                break;
            case rightGear:
                ckDriveTrain.resetSensors();
                ckLED.visionOn();
                end = System.currentTimeMillis() + 1450;
                while (isAutonomous() && System.currentTimeMillis() < end) {
                    ckDriveTrain.ckDrive.arcadeDrive(RobotMap.visionForward, 0);
                }
                if (!isAutonomous()) break;
                ckDriveTrain.ckDrive.stopMotor();
                dumbTurn(-60);
                ckDriveTrain.ckDrive.stopMotor();
                if (!isAutonomous()) break;
                gear();
                ckDriveTrain.ckDrive.stopMotor();
                if (!isAutonomous()) break;
                dumbTurn(0);
                ckDriveTrain.ckDrive.stopMotor();
                if (!isAutonomous()) break;
                end = System.currentTimeMillis() + 3500;
                while (isAutonomous() && System.currentTimeMillis() < end) {
                    ckDriveTrain.ckDrive.arcadeDrive(RobotMap.visionForward, 0);
                }
                break;
            case centerGear:
                gear();
                ckDriveTrain.ckDrive.stopMotor();
                ckLED.visionOff();
                break;
            default:
                break;
        }
        ckDriveTrain.ckDrive.stopMotor();
        ckLED.visionOff();
        imgProcReq = false;
        while (isAutonomous()) {
            Timer.delay(0.1);
        }
    }

    public void dumbTurn(double target) {
        double error = target - ckDriveTrain.ckNavX.getAngle();
        while (Math.abs(error - ckDriveTrain.ckNavX.getAngle()) > RobotMap.dumbTurnTolerance && isAutonomous()) {
            boolean left = error < 0;
            ckDriveTrain.ckDrive.arcadeDrive(0, (left ? 1D : -1D) * RobotMap.dumbTurn);
            Timer.delay(0.05);
            error = target - ckDriveTrain.ckNavX.getAngle();
        }
    }

    private void gear() {
        imgProcReq = true;
        runningThread = ckDriveTrain.driveVision();
        runningThread.runSync();
        ckDriveTrain.ckDrive.stopMotor();
        imgProcReq = false;
        ckGearArm.firePiston();
        Timer.delay(0.25);
        ckDriveTrain.teleDrive(-RobotMap.visionForward, 0);
        Timer.delay(1);
        ckDriveTrain.ckDrive.stopMotor();
        ckGearArm.closePiston();
    }

    @Override
    public void testInit() {
        teleopInit();
    }

    @Override
    public void testPeriodic() {
    }

    public static class DaemonThread extends Thread {
        public DaemonThread(Runnable r) {
            super(r);
            setDaemon(true);
        }
    }
}
