package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.Solenoid;

public class LED {
    private Solenoid visionLED;
    private Solenoid redLED;
    private Solenoid blueLED;
    private boolean visionStatus;
    private boolean redStatus;
    private boolean blueStatus;

    public LED() {
        visionLED = new Solenoid(RobotMap.pcmLEDCamera);
        redLED = new Solenoid(RobotMap.pcmLEDRed);
        blueLED = new Solenoid(RobotMap.pcmLEDBlue);
        visionStatus = false;
        redStatus = false;
        blueStatus = false;
    }

    //**** Vision LED
    public void visionOn() {
        visionLED.set(true);
        visionStatus = true;
    }

    public void visionOff() {
        visionLED.set(false);
        visionStatus = false;
    }

    public void toggleVision() {
        if (visionStatus) {
            visionOff();
        } else {
            visionOn();
        }
    }

    //**** Red LED
    public void redOn() {
        redLED.set(true);
        redStatus = true;
    }

    public void redOff() {
        redLED.set(false);
        redStatus = false;
    }

    public void toggleRed() {
        if (redStatus) {
            redOff();
        } else {
            redOn();
        }
    }

    //**** Blue LED
    public void blueOn() {
        blueLED.set(true);
        blueStatus = true;
    }

    public void blueOff() {
        blueLED.set(false);
        blueStatus = false;
    }

    public void toggleBlue() {
        if (blueStatus) {
            blueOff();
        } else {
            blueOn();
        }
    }


}


