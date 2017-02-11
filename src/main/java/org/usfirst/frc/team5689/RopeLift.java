package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.VictorSP;

/**
 * Created by Austin on 2017-01-30.
 */
public class RopeLift {
    VictorSP liftMotor;

    public RopeLift(){
        liftMotor = new VictorSP(RobotMap.pwmRopeMotor);
        liftMotor.setInverted(true);
    }


    public void testLift(double speed){
        liftMotor.set(speed);
    }

    public void stopLift(){
        liftMotor.stopMotor();
    }
}
