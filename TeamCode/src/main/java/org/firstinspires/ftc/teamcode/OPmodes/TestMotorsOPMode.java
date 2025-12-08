package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

@TeleOp
public class TestMotorsOPMode extends LinearOpMode {

    private int newTagId = 0;
    double currentrpm = 4000;

    @Override
    public void runOpMode() {

        final TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap, telemetry);
        final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hardwareMap);
        final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, newTagId);

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        DcMotorEx shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        CRServo fireer = hardwareMap.get(CRServo.class, "wheelIndexerMotor");
        CRServo turret = hardwareMap.get(CRServo.class, "turretServo");

        int lastTagId = 0;

        limelightSubsystem.startLimelight();

    waitForStart();
    while (opModeIsActive()) {
        LLResult result = limelightSubsystem.getLatestResult();
        List<LLResultTypes.FiducialResult> apriltagResult = result.getFiducialResults();

        telemetry.addData("currentrpm", currentrpm);
        telemetry.addData("ty", result.getTy());
        telemetry.addData("rpm", shooterMotor.getVelocity());
        telemetry.update();
        newTagId = lastTagId;
        for (LLResultTypes.FiducialResult detection : apriltagResult) {
            if (detection.getFiducialId() >= 21 && detection.getFiducialId() <= 23) {
                newTagId = detection.getFiducialId();
                break;
            }
        }
        if (gamepad1.a) {
                shooterMotor.setVelocity(currentrpm);
            }
        if (gamepad1.dpad_left) {
            shooterMotor2.setVelocity(currentrpm);
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
