package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp
public class TestMotorsOPMode extends LinearOpMode {



    @Override
    public void runOpMode() {

        final TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap, telemetry);
        final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hardwareMap);

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        CRServo fireer = hardwareMap.get(CRServo.class, "wheelIndexerMotor");
        CRServo turret = hardwareMap.get(CRServo.class, "turretServo");

    waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                shooterSubsystem.startShooting();
            }
            if (gamepad1.dpad_down) {
                shooterSubsystem.stopShooter();
            }
            if (gamepad1.b) {
                fireer.setPower(1);
            }
            if (gamepad1.y) {
                turret.setPower(1);
            }
            if (gamepad1.x) {
                turret.setPower(-1);
            }
        }
    }
}
