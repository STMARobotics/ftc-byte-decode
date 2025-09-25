package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {

    private final DcMotor frontRight;
    private final DcMotor rearRight;
    private final DcMotor rearLeft;
    private final DcMotor frontLeft;

    public DriveTrainSubsystem(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }


    public void moveDrivetrain(
            double x,
            double y,
            double rx) {

        double frontRightPower = (y + x + rx);
        double rearRightPower = (y - x + rx);
        double rearLeftPower = (y + x - rx);
        double frontLeftPower = (y - x - rx);



        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);
    }

    public void stopDrivetrain() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }
}
