package org.firstinspires.ftc.teamcode.OPmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShootingSubsystem;

@TeleOp
public class TeleOPMode extends CommandOpMode {

    @Override
    public void initialize() {

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        ShootingSubsystem shootingSubsystem = new ShootingSubsystem(hardwareMap, telemetry);

        FunctionalCommand teleopDriveCommand = new FunctionalCommand(driveTrainSubsystem::startTeleop,
                () -> driveTrainSubsystem.drive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        1),
                (b) -> driveTrainSubsystem.stopDrivetrain(),
                () -> false,
                driveTrainSubsystem);

        Pose startPose = new Pose(60, 8.000, Math.toRadians(90));
        PathChain path = driveTrainSubsystem.pathBuilder()
                .addPath(new BezierLine(new Pose(60, 8.000), new Pose(60, 60)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                        .build();

        register(driveTrainSubsystem, intakeSubsystem, shootingSubsystem);

        FollowPathCommand followPathCommand =
                new FollowPathCommand(startPose, path, driveTrainSubsystem)
                        .withGlobalMaxPower(0.5);

        StartEndCommand intakeCommand =
                new StartEndCommand(intakeSubsystem::runIntakeMotor, intakeSubsystem::stop, intakeSubsystem);

        InstantCommand resetPositionCommand =
                new InstantCommand(driveTrainSubsystem::resetLocalization, driveTrainSubsystem);

        new Trigger(() -> gamepad1.right_bumper).toggleWhenActive(intakeCommand);

        new Trigger(() -> gamepad1.b).whileActiveOnce(followPathCommand);

        new Trigger(() -> gamepad1.x).whenActive(resetPositionCommand);

        new Trigger(() -> gamepad1.right_trigger > 0.1).whileActiveContinuous(new ShootCommand(shootingSubsystem, intakeSubsystem, false));
                driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);

        new RunCommand(() ->
                telemetry.addData("isScheduled", followPathCommand.isScheduled())).schedule();
        new RunCommand(telemetry::update).schedule();

    }
}