package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ForkliftSubsystem;

public class ClimbCommand extends CommandBase {

    private final ForkliftSubsystem forkliftSubsystem;

    public ClimbCommand(ForkliftSubsystem forkliftSubsystem) {
        this.forkliftSubsystem = forkliftSubsystem;
    }

    public void initialize() {

    }

    public void execute() {

    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {

    }
}
