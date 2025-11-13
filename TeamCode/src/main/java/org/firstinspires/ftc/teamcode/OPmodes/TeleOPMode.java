package org.firstinspires.ftc.teamcode.OPmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

@TeleOp
public class TeleOPMode extends CommandOpMode {

    private int newTagId = 0;

    @Override
    public void initialize() {
        int lastTagId = 0;

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap, telemetry);
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, newTagId);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        IndexerSubsystem indexerSubsystem = new IndexerSubsystem(hardwareMap);
        TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap, limelightSubsystem);

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

        Pose startPose = new Pose(60, 8.000, Math.toRadians(90));
        PathChain path = driveTrainSubsystem.pathBuilder()
                .addPath(new BezierLine(new Pose(60, 8.000), new Pose(60, 60)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(60, 60), new Pose(100, 60)))
                .setTangentHeadingInterpolation()
                        .build();

        register(driveTrainSubsystem,
                limelightSubsystem,
                indexerSubsystem,
                intakeSubsystem,
                turretSubsystem);

        FollowPathCommand followPathCommand =
                new FollowPathCommand(startPose, path, driveTrainSubsystem)
                        .withGlobalMaxPower(0.5);

        FunctionalCommand teleopDriveCommand = new FunctionalCommand(driveTrainSubsystem::startTeleop,
                () -> driveTrainSubsystem.drive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        1),
                (b) -> driveTrainSubsystem.stopDrivetrain(),
                () -> false,
                driveTrainSubsystem);

        RunCommand intakeCommand =
                new RunCommand(intakeSubsystem::runIntakeMotor);

        RunCommand stopIntakeCommand =
                new RunCommand(intakeSubsystem::stop);

        InstantCommand resetPositionCommand =
                new InstantCommand(driveTrainSubsystem::resetLocalization, driveTrainSubsystem);

        new Trigger(() -> gamepad1.right_bumper).whenActive(intakeCommand);

        new Trigger(() -> gamepad1.left_bumper).whenActive(stopIntakeCommand);

        new Trigger(() -> gamepad1.x).whenActive(resetPositionCommand);

        new Trigger(() -> gamepad1.right_trigger > 0.1).whileActiveContinuous(
                 new ShootCommand(indexerSubsystem, limelightSubsystem, turretSubsystem));

        driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);

        new RunCommand(() ->
                telemetry.addData("isScheduled", followPathCommand.isScheduled())).schedule();
        new RunCommand(telemetry::update).schedule();
    }
}