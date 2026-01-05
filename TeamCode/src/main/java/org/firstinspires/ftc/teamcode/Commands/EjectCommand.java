package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

/**
 * Command that ejects balls from the robot by reversing the intake and indexer.
 * <p>
 * This command runs the intake, belt, and wheel in reverse to clear any jammed
 * or unwanted balls from the system. It continues running until interrupted.
 */
public class EjectCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    /**
     * Constructs an EjectCommand with the required subsystems.
     *
     * @param intakeSubsystem  the intake subsystem for reversing ball collection
     * @param indexerSubsystem the indexer subsystem for reversing ball feeding
     */
    public EjectCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    /**
     * Reverses all ball handling mechanisms to eject balls from the robot.
     */
    public void execute() {
        intakeSubsystem.reverse();
        indexerSubsystem.reverseBelt();
        indexerSubsystem.reverseWheel();
    }

    /**
     * Stops all ball handling mechanisms when the command ends.
     *
     * @param interrupted {@code true} if the command was interrupted, {@code false} if it ended normally
     */
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        indexerSubsystem.stopWheel();
        indexerSubsystem.stopBelt();
    }
}
