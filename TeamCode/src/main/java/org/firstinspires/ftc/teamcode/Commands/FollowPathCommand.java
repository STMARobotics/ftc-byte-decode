package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;

/**
 * Command that follows a predefined path using the Pedro Pathing library.
 * <p>
 * This command utilizes the drivetrain's follower to navigate along a {@link PathChain}
 * from a specified starting pose. It supports configurable maximum power and
 * optional position holding at the end of the path.
 */
public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final PathChain pathChain;
    private final Pose startPose;
    private final DriveTrainSubsystem drivetrainSubsystem;

    private boolean holdEnd = false;
    private double maxPower = 1.0;

    /**
     * Constructs a FollowPathCommand with the required path and subsystem.
     *
     * @param startPose           the starting pose for the path
     * @param pathChain           the path chain to follow
     * @param drivetrainSubsystem the drivetrain subsystem containing the follower
     */
    public FollowPathCommand(
            Pose startPose,
            PathChain pathChain,
            DriveTrainSubsystem drivetrainSubsystem) {
        this.follower = drivetrainSubsystem.getFollower();
        this.pathChain = pathChain;
        this.startPose = startPose;
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    /**
     * Sets the global maximum power for path following.
     *
     * @param globalMaxPower the maximum power value (0.0 to 1.0)
     * @return this command instance for method chaining
     */
    public FollowPathCommand withGlobalMaxPower(double globalMaxPower) {
        maxPower = globalMaxPower;
        return this;
    }

    /**
     * Configures whether the robot should hold its position at the end of the path.
     *
     * @param holdEnd {@code true} to hold position at end, {@code false} otherwise
     * @return this command instance for method chaining
     */
    public FollowPathCommand withHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Initializes the path following by setting the starting pose and beginning
     * to follow the configured path chain.
     */
    @Override
    public void initialize() {
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
        follower.setMaxPower(maxPower);
        if (maxPower != 1.0) {
            follower.followPath(pathChain, maxPower, holdEnd);
        }
        follower.followPath(pathChain, holdEnd);
    }

    /**
     * Called repeatedly while the command is scheduled.
     * Path following is handled internally by the follower.
     */
    @Override
    public void execute() {
    }

    /**
     * Determines if the command has finished executing.
     *
     * @return {@code true} if the follower is no longer busy, {@code false} otherwise
     */
    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    /**
     * Stops the drivetrain when the command ends.
     *
     * @param interrupted {@code true} if the command was interrupted, {@code false} if it ended normally
     */
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopDrivetrain();
    }
}