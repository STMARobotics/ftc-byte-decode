package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;

public class AutoCommand extends CommandBase {

    private final DriveTrainSubsystem driveTrainSubsystem;

    private final ElapsedTime timer = new ElapsedTime();
    private final String team;

    public AutoCommand(DriveTrainSubsystem driveTrainSubsystem, String team) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.team = team;

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.seconds() < 2) {
            driveTrainSubsystem.drive(.64, 0, 0);
        } else if (timer.seconds() < 3 && team == "Blue") {
            driveTrainSubsystem.drive(0, 0, .60);
        } else if (timer.seconds() < 3 && team == "Red") {
            driveTrainSubsystem.drive(0, 0, -.64);
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.seconds() >= 3);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stopDrivetrain();
    }
}