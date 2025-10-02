package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;

public class FollowPathCommand extends CommandBase {

    private final Follower follower;
    private final PathChain pathChain;
    private final Pose startPose;
    private final DriveTrainSubsystem drivetrainSubsystem;

    private boolean holdEnd = false;
    private double maxPower = 1.0;

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

    public FollowPathCommand withGlobalMaxPower(double globalMaxPower) {
        maxPower = globalMaxPower;
        return this;
    }

    public FollowPathCommand withHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

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

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopDrivetrain();
    }
}