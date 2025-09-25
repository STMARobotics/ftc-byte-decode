package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ManipulationSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorSubsystem;

import java.util.List;

@TeleOp
public class TeleOPMode extends LinearOpMode {

    private DriveTrainSubsystem driveTrainSubsystem;
    private SensorSubsystem sensorSubsystem;
    private ManipulationSubsystem manipulationSubsystem;
    private int lastTagId = 0;
    private boolean isIntaking = false;
    private ElapsedTime timer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState launchState;

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                manipulationSubsystem.runIndexer(30);
                if (manipulationSubsystem.getShooterSpeed() > 29) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                manipulationSubsystem.runIndexer(1);
                timer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (timer.seconds() > 2) {
                    launchState = LaunchState.IDLE;
                    manipulationSubsystem.stopShooter();
                    manipulationSubsystem.stopIndexer();
                }
                break;
        }
    }

    @Override
    public void runOpMode() {

            driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);
            sensorSubsystem = new SensorSubsystem(hardwareMap);
            manipulationSubsystem = new ManipulationSubsystem(hardwareMap);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            sensorSubsystem.startLimelight();
            waitForStart();

            while (opModeIsActive()) {
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

                double lefty = -gamepad1.left_stick_y;
                double leftx = gamepad1.left_stick_x;
                double rightx = gamepad1.right_stick_x;

                driveTrainSubsystem.moveDrivetrain(leftx, lefty, rightx);

                if (gamepad1.right_trigger > 0.1) {
                    launch(true);
                }

                if (gamepad1.left_trigger > 0.1) {
                    if (isIntaking) {
                        manipulationSubsystem.stopIndexer();
                    } else {
                        manipulationSubsystem.runIntake(1);
                    }
                }

            }
        }
}

