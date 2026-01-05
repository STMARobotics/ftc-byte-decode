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

/**
 * Testing OpMode for tuning and testing shooter motors, turret, and indexer.
 * <p>
 * This OpMode provides manual control for testing various mechanisms:
 * <ul>
 *   <li>A button: Run shooter motors at the current RPM setting</li>
 *   <li>D-pad up/down: Increase/decrease target RPM by 100</li>
 *   <li>B button: Run the indexer wheel servo</li>
 *   <li>Y button: Rotate turret in positive direction</li>
 *   <li>X button: Rotate turret in negative direction</li>
 * </ul>
 * Telemetry displays PID P value, target RPM, and actual RPM.
 * </p>
 */
@TeleOp
public class TestMotorsOPMode extends LinearOpMode {

    /**
     * Stores the ID of the detected AprilTag.
     */
    private int newTagId = 0;

    /**
     * The current target RPM for the shooter motors.
     */
    double currentrpm = 2000;

    /**
     * Main execution method for the OpMode.
     * <p>
     * Initializes subsystems and hardware, configures PIDF coefficients for
     * the shooter motors, and provides a control loop for manual testing of:
     * <ul>
     *   <li>Shooter velocity control with adjustable RPM</li>
     *   <li>Indexer wheel activation</li>
     *   <li>Turret rotation in both directions</li>
     * </ul>
     * Telemetry is continuously updated with current motor performance data.
     * </p>
     */
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
