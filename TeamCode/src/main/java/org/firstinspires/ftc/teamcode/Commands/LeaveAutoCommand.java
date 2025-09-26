package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ManipulationSubsystem;

public class LeaveAutoCommand extends CommandBase {

    private final DriveTrainSubsystem driveTrainSubsystem;
    private final ElapsedTime timer = new ElapsedTime();

    public LeaveAutoCommand(DriveTrainSubsystem driveTrainSubsystem) {
        this.driveTrainSubsystem = driveTrainSubsystem;

        addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        driveTrainSubsystem.drive(1, 1, 0);
    }

    @Override
    public boolean isFinished() {
        return (timer.seconds() > 5);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stopDrivetrain();
    }
}
