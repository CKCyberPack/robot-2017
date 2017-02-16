package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.Solenoid;

public class LED {
    private Solenoid visionLED;
    private Solenoid redLED;
    private Solenoid blueLED;
    private boolean visionStatus;
    private boolean redStatus;
    private boolean blueStatus;
    private int blinkCount;

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

    public void blinkPatternP() {
        blinkCount++;
        switch (blinkCount) {
            case 1:
            case 5:
            case 9:
                redOn();
            case 3:
            case 7:
            case 11:
                redOff();
            case 15:
            case 19:
            case 23:
                blueOn();
            case 17:
            case 21:
            case 25:
                blueOff();
            case 30:
                blinkCount = 0;
        }

    }
}


