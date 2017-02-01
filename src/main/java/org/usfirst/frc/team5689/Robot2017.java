package org.usfirst.frc.team5689;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class Robot2017 extends IterativeRobot {

    public static final int VISION_WIDTH = 320;
    public static final int VISION_HEIGHT = 240;

    private VisionThread visionThread;
    private double centerX = 0;
    private final Object imgLock = new Object();

    public void robotInit() {
        UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
        cam.setResolution(VISION_WIDTH, VISION_HEIGHT);
        cam.setFPS(2);

        visionThread = new VisionThread(cam, new GripPipeline(), pipeline -> {
            for (MatOfPoint p : pipeline.filterContoursOutput()) {
                Rect r = Imgproc.boundingRect(p);
                SmartDashboard.putNumber("X: ", r.x);
                SmartDashboard.putNumber("Y: ", r.y);
                SmartDashboard.putNumber("Width: ", r.width);
                SmartDashboard.putNumber("Height: ", r.height);
            }
        });
        visionThread.start();
    }

}
