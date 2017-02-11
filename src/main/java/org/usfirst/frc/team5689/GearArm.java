package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Created by Austin on 2017-01-30.
 */
public class GearArm {
    Solenoid armPiston;

    public GearArm(){
        armPiston = new Solenoid(RobotMap.pcmPiston);
    }

    public void firePiston(){
      armPiston.set(true);
    };

    public void closePiston(){
        armPiston.set(false);
    }

}
