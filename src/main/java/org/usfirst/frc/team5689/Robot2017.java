package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.*;

import static org.usfirst.frc.team5689.DriveRunnable.Status.*;

public class Robot2017 extends IterativeRobot {

    RobotDrive drive;
    XboxController controller;

    public void robotInit() {
        drive = new RobotDrive(0, 1, 2, 3);
        controller = new XboxController(1);
    }

    public void teleopPeriodic() {
        drive.arcadeDrive(controller.getY(GenericHID.Hand.kLeft), controller.getX(GenericHID.Hand.kRight));
    }

}
