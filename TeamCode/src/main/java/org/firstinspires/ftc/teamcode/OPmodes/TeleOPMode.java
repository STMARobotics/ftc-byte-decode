package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorSubsystem;

import java.util.List;

@TeleOp
public class TeleOPMode extends CommandOpMode {

    private DriveTrainSubsystem driveTrainSubsystem;
    private SensorSubsystem sensorSubsystem;

    @Override
    public void initialize() {

    driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);
    sensorSubsystem = new SensorSubsystem(hardwareMap);
    sensorSubsystem.startLimelight();

        RunCommand teleopDriveCommand = new RunCommand(() -> driveTrainSubsystem.moveDrivetrain(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x));


    public void runOpMode() {



            telemetry.addData("Status", "Initialized");
            telemetry.update();


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

                double frontRightPower = (lefty + leftx + rightx);
                double rearRightPower = (lefty - leftx + rightx);
                double rearLeftPower = (lefty + leftx - rightx);
                double frontLeftPower = (lefty - leftx - rightx);

                driveTrainSubsystem.moveDrivetrain(leftx, lefty, rightx);

                if (gamepad1.right_trigger > 0.1) {
                    schedule
                }

            }
        }

}

