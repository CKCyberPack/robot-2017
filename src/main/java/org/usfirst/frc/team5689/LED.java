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
    private boolean blinkStatus;

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
        Robot2017.imgProcReq = true;
    }

    public void visionOff() {
        visionLED.set(false);
        visionStatus = false;
        Robot2017.imgProcReq = false;
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
        blinkStatus = true;
        new Robot2017.DaemonThread(() -> {
            int c = 0;
            while (blinkStatus) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    break;
                }
                if (c % 2 == 0) {
                    blueOff();
                    redOff();
                } else {
                    if (c < 6)
                        redOn();
                    else if (c > 8)
                        blueOn();
                }
                if (c > 14){
                    c = 0;
                }else {
                    c++;
                }
            }
        }).start();
    }

    public void blinkOff(){
        blinkStatus = false;
    }
}


