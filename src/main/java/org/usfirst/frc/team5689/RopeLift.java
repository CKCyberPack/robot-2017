package org.usfirst.frc.team5689;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.VictorSP;

public class RopeLift {
    private VictorSP liftMotor;
    private boolean ropeOverCurrent;

    public RopeLift(){
        //Initialize Motor
        liftMotor = new VictorSP(RobotMap.pwmRopeMotor);
        liftMotor.setInverted(true);

        //Set Variables
        ropeOverCurrent = false;
    }


    public void safeClimb(double speed, PowerDistributionPanel pdp){
        //Check if we already went over current
        if (ropeOverCurrent){
            liftMotor.stopMotor();
        }else{
            liftMotor.set(speed);

            //Check for over current.
            if (pdp.getCurrent(RobotMap.pdpRopeMotor) > RobotMap.ropeOverCurrent){
                ropeOverCurrent = true;
            }
        }

    }

    public void dangerClimb(double speed){
        liftMotor.set(speed);
    }

    public void clearOverCurrent(){
        ropeOverCurrent = false;
    }
}
