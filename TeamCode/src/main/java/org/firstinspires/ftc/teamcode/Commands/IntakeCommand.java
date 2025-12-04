package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    public void execute() {
        if (intakeSubsystem.isSensorTripped() && indexerSubsystem.isBeltSensorTripped() && indexerSubsystem.isWheelSensorTripped()) {
            intakeSubsystem.stop();
        } else {
            intakeSubsystem.runIntakeMotor();
        }

        if (indexerSubsystem.isBeltSensorTripped() && indexerSubsystem.isWheelSensorTripped()) {
            indexerSubsystem.stopBelt();
        } else {
            indexerSubsystem.runBelt();
        }

        if (indexerSubsystem.isWheelSensorTripped()) {
            indexerSubsystem.stopWheel();
        } else {
            indexerSubsystem.runWheel();
        }

    }

    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        indexerSubsystem.stopWheel();
        indexerSubsystem.stopBelt();
    }
}
