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
    public void initialize() {
        hasShot = false;
    }

    @Override
    public void execute() {
        manipulationSubsystem.runShooter();
        if (manipulationSubsystem.getShooterSpeed() >= 30 && !hasShot) {
            timer.reset();
            manipulationSubsystem.runIndexer(1);
            hasShot = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (hasShot && timer.seconds() >= .25);
    }

    @Override
    public void end(boolean interrupted) {
        manipulationSubsystem.stopShooter();
        manipulationSubsystem.stopIndexer();
    }
}
