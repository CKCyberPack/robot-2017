package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;

public class LED {
    private Solenoid visionLED;
    private Relay redLED;
    private Relay blueLED;
    private boolean visionStatus;
    private boolean redStatus;
    private boolean blueStatus;
    private int blinkCount;

    public LED() {
        visionLED = new Solenoid(RobotMap.pcmLEDCamera);
        redLED = new Relay(RobotMap.relayRed, Relay.Direction.kBoth);
        blueLED = new Relay(RobotMap.relayBlue, Relay.Direction.kBoth);
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
        redLED.set(Relay.Value.kOn);
        redStatus = true;
    }

    public void redOff() {
        redLED.set(Relay.Value.kOff);
        redStatus = false;
    }

    public void redFront(){
        redLED.set(Relay.Value.kForward);
        redStatus = true;
    }

    public void redBottom(){
        redLED.set(Relay.Value.kReverse);
        redStatus = true;
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
        blueLED.set(Relay.Value.kOn);
        blueStatus = true;
    }

    public void blueOff() {
        blueLED.set(Relay.Value.kOff);
        blueStatus = false;
    }

    public void blueFront(){
        blueLED.set(Relay.Value.kForward);
        blueStatus = true;
    }

    public void blueBottom(){
        blueLED.set(Relay.Value.kReverse);
        blueStatus = true;
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


