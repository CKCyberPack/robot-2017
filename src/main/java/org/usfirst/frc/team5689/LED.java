package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.Solenoid;

public class LED {
    private Solenoid visionLED;
    private boolean visionStatus;

    public LED() {
        visionLED = new Solenoid(RobotMap.pcmLEDCamera);
        visionStatus = false;
    }

    public void visonOn() {
        visionLED.set(true);
        visionStatus = true;
    }

    ;

    public void visionOff() {
        visionLED.set(false);
        visionStatus = false;
    }

    public void toggleVision() {
        if (visionStatus) {
            visonOn();
        } else {
            visionOff();
        }
    }
}


