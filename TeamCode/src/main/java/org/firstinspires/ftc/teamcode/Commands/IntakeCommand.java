package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

/**
 * Command that manages the intake and indexer to collect and stage balls.
 * <p>
 * This command uses sensor feedback to automatically control the intake,
 * belt, and wheel mechanisms. It stops each component when its corresponding
 * sensor detects a ball to prevent jamming and ensure proper ball staging.
 * The command continues running until interrupted.
 */
public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    /**
     * Constructs an IntakeCommand with the required subsystems.
     *
     * @param intakeSubsystem  the intake subsystem for collecting balls
     * @param indexerSubsystem the indexer subsystem for staging balls
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    /**
     * Manages the intake and indexer based on sensor states.
     * <p>
     * The logic works from the end of the path backward:
     * <ol>
     *   <li><b>Wheel control:</b> Stops when wheel sensor detects a ball (final staging position)</li>
     *   <li><b>Belt control:</b> Stops when both belt AND wheel sensors are tripped 
     *       (a ball is staged at wheel, and another is waiting on belt)</li>
     *   <li><b>Intake control:</b> Stops when ALL three sensors are tripped 
     *       (system is full - balls at intake, belt, and wheel positions)</li>
     * </ol>
     */
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

    /**
     * Stops all ball indexing mechanisms when the command ends.
     *
     * @param interrupted {@code true} if the command was interrupted, {@code false} if it ended normally
     */
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        indexerSubsystem.stopWheel();
        indexerSubsystem.stopBelt();
    }
}
