package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.Solenoid;

public class LED {
    Solenoid visionLED;

    public LED() {
        visionLED = new Solenoid(RobotMap.pcmLEDCamera);
    }

    public void onLED() {
        visionLED.set(true);
    }

    ;

    public void offLED() {
        visionLED.set(false);
    }

}
