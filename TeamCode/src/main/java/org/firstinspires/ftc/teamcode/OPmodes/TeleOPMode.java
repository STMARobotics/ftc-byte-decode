package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ManipulationSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorSubsystem;

import java.util.List;

@TeleOp
public class TeleOPMode extends CommandOpMode {

    @Override
    public void initialize() {
        int lastTagId = 0;

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);
        SensorSubsystem sensorSubsystem = new SensorSubsystem(hardwareMap);
        ManipulationSubsystem manipulationSubsystem = new ManipulationSubsystem(hardwareMap);

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
        telemetry.addData("Motif", sensorSubsystem.getMotif(newTagId));

        register(driveTrainSubsystem, manipulationSubsystem, sensorSubsystem);

        RunCommand teleopDriveCommand =
                new RunCommand(() -> driveTrainSubsystem.driveFieldRelative(
                        -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x), driveTrainSubsystem);

        RunCommand cancelIntakeCommand =
                new RunCommand(manipulationSubsystem::stopIntake, manipulationSubsystem);

        new Trigger(() -> gamepad1.right_trigger > 0.1).whileActiveContinuous(
                new ShootCommand(manipulationSubsystem));

        new Trigger(() -> gamepad1.left_bumper).whenActive(
                new IntakeCommand(manipulationSubsystem));

        new Trigger(() -> gamepad1.right_bumper).whenActive(cancelIntakeCommand);

        driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);
    }
}

