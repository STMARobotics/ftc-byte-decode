package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Subsystems.ManipulationSubsystem;

public class IntakeCommand extends CommandBase {

    private final ManipulationSubsystem manipulationSubsystem;

    public IntakeCommand(ManipulationSubsystem manipulationSubsystem) {
        this.manipulationSubsystem = manipulationSubsystem;

        addRequirements(manipulationSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        manipulationSubsystem.runIntake(1);
    }

    @Override
    public void end(boolean interrupted) {
        manipulationSubsystem.stopIntake();
    }
}
