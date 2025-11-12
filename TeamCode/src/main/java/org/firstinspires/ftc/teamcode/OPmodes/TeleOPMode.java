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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;

import java.util.List;

@TeleOp
public class TeleOPMode extends CommandOpMode {

    @Override
    public void initialize() {
        int lastTagId = 0;

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap, telemetry);
        LimelightSubsystem sensorSubsystem = new LimelightSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        sensorSubsystem.startLimelight();
        LLResult result = sensorSubsystem.getLatestResult();
        Pose3D pose = result.getBotpose();

        List<LLResultTypes.FiducialResult> apriltagResult = result.getFiducialResults();
        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("Status", "Running");

        int newTagId = lastTagId;
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


        register(driveTrainSubsystem, sensorSubsystem);

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

        new Trigger(() -> gamepad1.x).whenActive(resetPositionCommand);

        new Trigger(() -> gamepad1.left_bumper).whenActive(stopIntakeCommand);

        new Trigger(() -> gamepad1.right_trigger > 0.1).whileActiveContinuous(
                 followPathCommand);

        driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);

        new RunCommand(() ->
                telemetry.addData("isScheduled", followPathCommand.isScheduled())).schedule();
        new RunCommand(telemetry::update).schedule();
    }
}