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

import org.firstinspires.ftc.teamcode.Commands.EjectCommand;
import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

/**
 * TeleOp OpMode for the blue alliance.
 * <p>
 * This OpMode provides driver-controlled operation with the following features:
 * <ul>
 *   <li>Field-centric or robot-centric driving using gamepad joysticks</li>
 *   <li>AprilTag detection using the Limelight vision system</li>
 *   <li>Trigger-activated shooting with automatic turret aiming</li>
 *   <li>Trigger-activated intake and indexer control</li>
 *   <li>Manual turret adjustment using D-pad</li>
 *   <li>Game element ejection functionality</li>
 * </ul>
 * </p>
 */
@TeleOp
public class BlueTeleOPMode extends CommandOpMode {

    private int newTagId = 0;

    /**
     * Initializes the TeleOp routine.
     * <p>
     * This method performs the following setup operations:
     * <ul>
     *   <li>Creates and configures all robot subsystems</li>
     *   <li>Starts the Limelight and detects AprilTags</li>
     *   <li>Registers all subsystems with the command scheduler</li>
     *   <li>Configures gamepad button bindings:
     *     <ul>
     *       <li>Right trigger: Toggle shooting mode</li>
     *       <li>Left trigger: Toggle intake</li>
     *       <li>A button: Toggle eject</li>
     *       <li>Back button: Reset localization</li>
     *       <li>D-pad left: Manual turret control</li>
     *     </ul>
     *   </li>
     *   <li>Sets the default drive command for joystick control</li>
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
                if (detection.getFiducialId() == 24) {
                    newTagId = detection.getFiducialId();
                    break;
                }
            }
        }

        register(driveTrainSubsystem,
                limelightSubsystem,
                indexerSubsystem,
                intakeSubsystem,
                turretSubsystem,
                shooterSubsystem);

        FunctionalCommand teleopDriveCommand = new FunctionalCommand(driveTrainSubsystem::startTeleop,
                () -> driveTrainSubsystem.drive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        1),
                (b) -> driveTrainSubsystem.stopDrivetrain(),
                () -> false,
                driveTrainSubsystem);

        InstantCommand resetPositionCommand =
                new InstantCommand(driveTrainSubsystem::resetLocalization, driveTrainSubsystem);

        new Trigger(() -> gamepad1.right_trigger > 0.1).toggleWhenActive(
                new ShootCommand(
                        indexerSubsystem,
                        limelightSubsystem,
                        turretSubsystem,
                        shooterSubsystem,
                        intakeSubsystem,
                        telemetry));

        new Trigger(() -> gamepad1.back).toggleWhenActive(resetPositionCommand);

        new Trigger(() -> gamepad1.left_trigger > 0.1).toggleWhenActive(new IntakeCommand(intakeSubsystem, indexerSubsystem));

        new Trigger(() -> gamepad1.a).toggleWhenActive(new EjectCommand(intakeSubsystem, indexerSubsystem));

        new Trigger(() -> gamepad1.dpad_left).whileActiveContinuous(new RunCommand(() -> turretSubsystem.setTurretPower(1.0)));

        new Trigger(() -> gamepad1.dpad_left).whileActiveContinuous(new RunCommand(() -> turretSubsystem.setTurretPower(-1.0)));

        driveTrainSubsystem.setDefaultCommand(teleopDriveCommand);

        new RunCommand(telemetry::update).schedule();
    }
}
