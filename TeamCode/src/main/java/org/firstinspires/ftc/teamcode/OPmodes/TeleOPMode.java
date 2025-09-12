package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorSubsystem;

@TeleOp
public class TeleOPMode extends LinearOpMode {

    private IMU imu;

    private DriveTrainSubsystem driveTrainSubsystem;
    private SensorSubsystem sensorSubsystem;

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(IMU.class, "imu");
        
        driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);
        sensorSubsystem = new SensorSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pose = SensorSubsystem.getPose2d();

            telemetry.addData("Status", "Running");
            telemetry.update();
            telemetry.addData("X coordinate", pose.x);
            telemetry.addData("Y coordinate", pose.y);
            telemetry.addData("Heading angle", pose.h);double lefty = -gamepad1.left_stick_y;

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
