package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@TeleOp
public class TestMotorsOPMode extends LinearOpMode {

    private int newTagId = 0;
    double currentrpm = 2000;

    @Override
    public void runOpMode() {

        final TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap, telemetry);
        final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hardwareMap);
        final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, "red");

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        DcMotorEx shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        CRServo fireer = hardwareMap.get(CRServo.class, "wheelIndexerMotor");
        CRServo turret = hardwareMap.get(CRServo.class, "turretServo");

        PIDFCoefficients pid = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pid.p = 7;
        PIDFCoefficients pid2 = shooterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        pid2.p = 7;

        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid2);

        int lastTagId = 0;

        limelightSubsystem.startLimelight();

    waitForStart();
    while (opModeIsActive()) {

        telemetry.addData("p", pid.p);
        telemetry.addData("currentrpm", currentrpm);
        telemetry.addData("rpm", shooterMotor.getVelocity()/28*60);
        telemetry.update();
        if (gamepad1.a) {
                shooterMotor.setVelocity(currentrpm);
                shooterMotor2.setVelocity(currentrpm);
            }
        if (gamepad1.dpad_left) {
        }
            if (gamepad1.dpad_up) {
                currentrpm += 100;
                sleep(300);
            }
            if (gamepad1.dpad_down) {
                currentrpm -= 100;
                sleep(300);
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
