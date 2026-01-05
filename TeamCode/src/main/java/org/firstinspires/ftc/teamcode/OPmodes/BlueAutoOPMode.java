package org.firstinspires.ftc.teamcode.OPmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

/**
 * Autonomous OpMode for the blue alliance.
 * <p>
 * This OpMode executes a predefined autonomous routine that includes:
 * <ul>
 *   <li>Detecting AprilTags using the Limelight vision system</li>
 *   <li>Following a sequence of paths using Bezier curves</li>
 *   <li>Intaking game elements while driving</li>
 *   <li>Automatically shooting at detected targets</li>
 * </ul>
 * </p>
 */
@Autonomous
public class BlueAutoOPMode extends CommandOpMode {

    private int newTagId = 0;

    /**
     * Initializes the autonomous routine.
     * <p>
     * This method performs the following setup operations:
     * <ul>
     *   <li>Creates and configures all robot subsystems</li>
     *   <li>Starts the Limelight
     *   <li>Builds path chains for the robot to follow</li>
     *   <li>Registers all subsystems with the command scheduler</li>
     *   <li>Schedules the autonomous command sequence</li>
     * </ul>
     * </p>
     */
    @Override
    public void initialize() {
        int lastTagId = 0;

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap, telemetry);
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, "blue");
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        IndexerSubsystem indexerSubsystem = new IndexerSubsystem(hardwareMap, telemetry);
        TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hardwareMap);

        limelightSubsystem.startLimelight();
        LLResult result = limelightSubsystem.getLatestResult();
        List<LLResultTypes.FiducialResult> apriltagResult = result.getFiducialResults();

        newTagId = lastTagId;
        for (LLResultTypes.FiducialResult detection : apriltagResult) {
            if (detection.getFiducialId() >= 21 && detection.getFiducialId() <= 23) {
                newTagId = detection.getFiducialId();
                break;
            }
        }

        Pose firstStartPose = new Pose(63, 9.000, Math.toRadians(180));
        PathChain firstPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63, 9), new Pose(10, 24))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        Pose secondStartPose = new Pose(10, 24, Math.toRadians(270));
        PathChain secondPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10, 24), new Pose(10, 10.3))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))

                .build();

        Pose thirdStartPose = new Pose(10, 10.3, Math.toRadians(270));
        PathChain thirdPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10, 10.3), new Pose(40, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        Pose fourthStartPose = new Pose(40, 12, Math.toRadians(180));
        PathChain fourthPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40, 12), new Pose(45, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        register(driveTrainSubsystem,
                limelightSubsystem,
                indexerSubsystem,
                intakeSubsystem,
                turretSubsystem,
                shooterSubsystem);

        FollowPathCommand followFirstPathCommand =
                new FollowPathCommand(firstStartPose, firstPath, driveTrainSubsystem)
                        .withGlobalMaxPower(.8);

        FollowPathCommand followSecondPathCommand =
                new FollowPathCommand(secondStartPose, secondPath, driveTrainSubsystem)
                        .withGlobalMaxPower(.2);

        FollowPathCommand followThirdPathCommand =
                new FollowPathCommand(thirdStartPose, thirdPath, driveTrainSubsystem)
                        .withGlobalMaxPower(.8);

        FollowPathCommand followFourthPathCommand =
                new FollowPathCommand(fourthStartPose, fourthPath, driveTrainSubsystem)
                        .withGlobalMaxPower(0.8);

        AutoShootCommand shootCommand = new AutoShootCommand(
                indexerSubsystem, limelightSubsystem, turretSubsystem, shooterSubsystem, intakeSubsystem, telemetry);

        SequentialCommandGroup autoCommand = new SequentialCommandGroup(
                shootCommand,
                new ParallelDeadlineGroup(
                        followFirstPathCommand, new IntakeCommand(intakeSubsystem, indexerSubsystem)),
                new ParallelDeadlineGroup(
                        followSecondPathCommand, new IntakeCommand(intakeSubsystem, indexerSubsystem)),
                new ParallelDeadlineGroup(
                        followThirdPathCommand, new IntakeCommand(intakeSubsystem, indexerSubsystem)),
                shootCommand,
                new ParallelDeadlineGroup(
                        followFourthPathCommand, new IntakeCommand(intakeSubsystem, indexerSubsystem))
        );

        new RunCommand(telemetry::update).schedule();
        autoCommand.schedule();
    }

}
