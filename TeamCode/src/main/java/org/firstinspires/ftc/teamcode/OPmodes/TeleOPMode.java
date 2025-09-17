package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ForkliftSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.StagingSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
public class TeleOPMode extends LinearOpMode {

    private IMU imu;

    private DriveTrainSubsystem driveTrainSubsystem;
    private SensorSubsystem sensorSubsystem;
    private ForkliftSubsystem forkliftSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private StagingSubsystem stagingSubsystem;
    private TurretSubsystem turretSubsystem;
    private DcMotor motor;
    private int lastTagId = 0;

    @Override
    public void runOpMode() {

      /*  List<AprilTagDetection> detections = sensorSubsystem.getDetections();

        int newTagId = lastTagId;
        for (AprilTagDetection detection : detections) {
            if (detection.id >= 21 && detection.id <= 23) {
                // Found an Obelisk tag
                newTagId = detection.id;
                break;
            }
*/
            driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);
            sensorSubsystem = new SensorSubsystem(hardwareMap);
            forkliftSubsystem = new ForkliftSubsystem(hardwareMap);
            indexerSubsystem = new IndexerSubsystem(hardwareMap);
            intakeSubsystem = new IntakeSubsystem(hardwareMap);
            stagingSubsystem = new StagingSubsystem(hardwareMap);
            turretSubsystem = new TurretSubsystem(hardwareMap);

            motor = hardwareMap.get(DcMotor.class, "motor");

            telemetry.addData("Status", "Initialized");
            telemetry.update();
            sensorSubsystem.startLimelight();
            waitForStart();

            while (opModeIsActive()) {
                SparkFunOTOS.Pose2D pose = SensorSubsystem.getPose2d();

                motor.setPower(.9);

                telemetry.addData("Status", "Running");
                telemetry.update();
                telemetry.addData("X coordinate", pose.x);
                telemetry.addData("Y coordinate", pose.y);
                telemetry.addData("Heading angle", pose.h);
                double lefty = -gamepad1.left_stick_y;
              //  telemetry.addData("Motif", sensorSubsystem.getDetections());

                double leftx = gamepad1.left_stick_x;
                double rightx = gamepad1.right_stick_x;

                double frontRightPower = (lefty + leftx + rightx);
                double rearRightPower = (lefty - leftx + rightx);
                double rearLeftPower = (lefty + leftx - rightx);
                double frontLeftPower = (lefty - leftx - rightx);

                driveTrainSubsystem.moveDrivetrain(
                        frontRightPower,
                        rearRightPower,
                        frontLeftPower,
                        rearLeftPower);

            }
        }
    }
//}
