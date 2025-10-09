package org.firstinspires.ftc.teamcode.OPmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

import java.util.List;

@TeleOp
public class TeleOPMode extends CommandOpMode {

    @Override
    public void initialize() {
        int lastTagId = 0;

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        Pose startPose = new Pose(60, 8.000, Math.toRadians(90));
        PathChain path = driveTrainSubsystem.pathBuilder()
                .addPath(new BezierLine(new Pose(60, 8.000), new Pose(60, 60)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(60, 60), new Pose(100, 60)))
                .setTangentHeadingInterpolation()
                        .build();


        register(driveTrainSubsystem);

        FollowPathCommand followPathCommand =
                new FollowPathCommand(startPose, path, driveTrainSubsystem)
                        .withGlobalMaxPower(0.5);

        RunCommand teleopDriveCommand =
                new RunCommand(() -> driveTrainSubsystem.driveFieldRelative(
                        -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x), driveTrainSubsystem);

        RunCommand intakeCommand =
                new RunCommand(() -> intakeSubsystem.runIntakeMotor());

        RunCommand stopIntakeCommand =
                new RunCommand(() -> intakeSubsystem.stop());

        new Trigger(() -> gamepad1.right_bumper).whenActive(intakeCommand);

        new Trigger(() -> gamepad1.left_bumper).whenActive(stopIntakeCommand);

        new Trigger(() -> gamepad1.right_trigger > 0.1).whileActiveContinuous(
                 followPathCommand);

        driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);
    }
}