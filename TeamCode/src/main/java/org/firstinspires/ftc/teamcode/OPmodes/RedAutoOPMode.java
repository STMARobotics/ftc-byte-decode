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
//import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

@Autonomous
public class RedAutoOPMode extends CommandOpMode {

    private int newTagId = 0;

    @Override
    public void initialize() {
        int lastTagId = 0;

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap, telemetry);
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, "red");
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

        Pose firstStartPose = new Pose(80, 9.000, Math.toRadians(0));
        PathChain firstPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(80, 9), new Pose(131.5, 24))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        Pose secondStartPose = new Pose(131.5, 24, Math.toRadians(-90));
        PathChain secondPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(131.5, 24), new Pose(131.5, 10.3))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                        .build();

        Pose thirdStartPose = new Pose(131.5, 10.3, Math.toRadians(-90));
        PathChain thirdPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(131.5, 10.3), new Pose(85, 12))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                        .build();

        Pose fourthStartPose = new Pose(85, 12, Math.toRadians(0));
        PathChain fourthPath = driveTrainSubsystem.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(85, 12), new Pose(103, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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
                        .withGlobalMaxPower(.4);

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
