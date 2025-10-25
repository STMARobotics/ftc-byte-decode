package org.firstinspires.ftc.teamcode.OPmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Commands.AutoCommand;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShootingSubsystem;

@Autonomous
public class RedAutoOPMode extends CommandOpMode {

    @Override
    public void initialize() {
        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        ShootingSubsystem shootingSubsystem = new ShootingSubsystem(hardwareMap, telemetry);

        Pose startPose = new Pose(60, 8.000, Math.toRadians(90));
        PathChain path = driveTrainSubsystem.pathBuilder()
                .addPath(new BezierLine(new Pose(60, 8.000), new Pose(60, 10)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(100))
                .build();

        register(driveTrainSubsystem, intakeSubsystem, shootingSubsystem);

        FollowPathCommand followPathCommand =
                new FollowPathCommand(startPose, path, driveTrainSubsystem)
                        .withGlobalMaxPower(0.5);

        RunCommand teleopDriveCommand =
                new RunCommand(() -> driveTrainSubsystem.driveFieldRelative(
                        -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x), driveTrainSubsystem);

        InstantCommand resetPositionCommand =
                new InstantCommand(driveTrainSubsystem::resetLocalization);

        new Trigger(() -> gamepad1.b).whileActiveOnce(followPathCommand);

        new Trigger(() -> gamepad1.x).whenActive(resetPositionCommand);

        new RunCommand(telemetry::update).schedule();

        Command autoCommand = new AutoCommand(driveTrainSubsystem, "Red").andThen(new ShootCommand(shootingSubsystem, intakeSubsystem, true)).andThen(new ShootCommand(shootingSubsystem, intakeSubsystem, true));

        new Trigger(this::opModeIsActive).whileActiveOnce(autoCommand);
    }
}
