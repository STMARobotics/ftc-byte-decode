package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.*;

@TeleOp(name="Test: Drive motors", group="Testing")
public class TestDrivetrainOPMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, FRONT_LEFT_MOTOR_NAME);
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, FRONT_RIGHT_MOTOR_NAME);
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, BACK_LEFT_MOTOR_NAME);
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, BACK_RIGHT_MOTOR_NAME);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {

            frontRightMotor.setPower(gamepad1.y ? .5 : 0);
            frontLeftMotor.setPower(gamepad1.x ? .5 : 0);
            backRightMotor.setPower(gamepad1.b ? .5 : 0);
            backLeftMotor.setPower(gamepad1.a ? .5 : 0);

        }
    }
}