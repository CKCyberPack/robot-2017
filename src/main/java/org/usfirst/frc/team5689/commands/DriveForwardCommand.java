package org.usfirst.frc.team5689.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team5689.subsystems.DriveSubsystem;

public class DriveForwardCommand extends Command {

    private double inches;
    private boolean stopOnCollision;

    public DriveForwardCommand(double inches, boolean stopOnCollision) {
        super("DriveForward " + inches);
        this.inches = inches;
        this.stopOnCollision = stopOnCollision;
        requires(DriveSubsystem.getInstance());
    }

    protected void initialize() {
        DriveSubsystem.getInstance().resetNav();
    }

    protected void execute() {
        DriveSubsystem.getInstance().driveForward(inches, stopOnCollision);
    }

    protected boolean isFinished() {
        return false;
    }
}
