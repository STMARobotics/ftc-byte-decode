package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants.*;

/**
 * Testing OpMode for individually testing each drivetrain motor.
 * <p>
 * This OpMode allows manual testing of each drive motor using gamepad buttons:
 * <ul>
 *   <li>Y button: Front right motor</li>
 *   <li>X button: Front left motor</li>
 *   <li>B button: Back right motor</li>
 *   <li>A button: Back left motor</li>
 * </ul>
 * Each motor runs at 50% power when its corresponding button is pressed.
 * </p>
 */
@TeleOp(name="Test: Drive motors", group="Testing")
public class TestDrivetrainOPMode extends LinearOpMode {

    /**
     * Main execution method for the OpMode.
     * <p>
     * Initializes all four drivetrain motors with appropriate directions,
     * then continuously monitors gamepad button presses to control each
     * motor individually for testing purposes.
     * </p>
     */
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