package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ManipulationSubsystem;

public class ShootCommand extends CommandBase {

    private final ManipulationSubsystem manipulationSubsystem;

    ElapsedTime timer = new ElapsedTime();

    private boolean hasShot = false;

    public ShootCommand(ManipulationSubsystem manipulationSubsystem) {
        this.manipulationSubsystem = manipulationSubsystem;

        addRequirements(manipulationSubsystem);
    }

    @Override
    public void execute() {
        manipulationSubsystem.runShooter(1);
        if (manipulationSubsystem.getShooterSpeed() >= 30 && !hasShot) {
            timer.reset();
            manipulationSubsystem.runIndexer(1);

        }
    }

    @Override
    public boolean isFinished() {
        return (hasShot && timer.time() >= 5);
    }

    @Override
    public void end(boolean interrupted) {
        manipulationSubsystem.stopShooter();
        manipulationSubsystem.stopIndexer();
    }
}
