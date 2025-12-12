package org.firstinspires.ftc.teamcode.OPmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

@Autonomous
public class AutoOPMode extends CommandOpMode {

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

        Pose startPose = new Pose(80, 9.000, Math.toRadians(90));
        PathChain path = driveTrainSubsystem.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(80, 9), new Pose(105, 35))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        register(driveTrainSubsystem,
                limelightSubsystem,
                indexerSubsystem,
                intakeSubsystem,
                turretSubsystem,
                shooterSubsystem);

        FollowPathCommand followPathCommand =
                new FollowPathCommand(startPose, path, driveTrainSubsystem)
                        .withGlobalMaxPower(1);

        RunCommand intakeCommand =
                new RunCommand(intakeSubsystem::runIntakeMotor);

        RunCommand stopIntakeCommand =
                new RunCommand(intakeSubsystem::stop);

        AutoShootCommand shootCommand = new AutoShootCommand(
                indexerSubsystem, limelightSubsystem, turretSubsystem, shooterSubsystem, intakeSubsystem, telemetry);

        SequentialCommandGroup autoCommand = new SequentialCommandGroup(shootCommand, followPathCommand);

        new RunCommand(() ->
                telemetry.addData("isScheduled", followPathCommand.isScheduled())).schedule();
        new RunCommand(telemetry::update).schedule();
        autoCommand.schedule();
    }

}
