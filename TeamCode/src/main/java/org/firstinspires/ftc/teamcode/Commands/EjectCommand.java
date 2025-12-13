package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class EjectCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public EjectCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    public void execute() {
        intakeSubsystem.reverse();
        indexerSubsystem.reverseBelt();
        indexerSubsystem.reverseWheel();
    }

    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        indexerSubsystem.stopWheel();
        indexerSubsystem.stopBelt();
    }
}
