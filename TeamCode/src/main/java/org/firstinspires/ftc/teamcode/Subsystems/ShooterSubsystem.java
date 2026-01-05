package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Subsystem that controls the shooter mechanism of the robot.
 * Uses two motors with velocity control for consistent shooting.
 */
public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;

    /**
     * Constructs a new ShooterSubsystem.
     * Configures two shooter motors with PIDF coefficients for velocity control.
     *
     * @param hardwareMap the hardware map from the OpMode
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid = new PIDFCoefficients();
        pid.p = 500;
        pid.i = 0;
        pid.d = 0;
        pid.f = 17;

        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid);

        shooterMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Starts the shooter motors at the specified velocity.
     *
     * @param speed the target velocity in encoder ticks per second
     */
    public void startShooting(double speed) {
        shooterMotor1.setVelocity(speed);
        shooterMotor2.setVelocity(speed);
    }

    /**
     * Stops both shooter motors.
     */
    public void stopShooter() {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
    }

    /**
     * Checks if the shooter has reached the target velocity and is ready to shoot.
     *
     * @param speed the target velocity to compare against
     * @return true if the shooter is within 75 ticks/sec of the target velocity
     */
    public boolean isReadyToShoot(double speed) {
        return (Math.abs(Math.abs(shooterMotor1.getVelocity()) - speed) / 2 < 75);
    }

    /**
     * Gets the current velocity of shooter motor 1.
     *
     * @return the velocity of shooter motor 1 in encoder ticks per second
     */
    public double getShooter1Velocity() {
        return shooterMotor1.getVelocity();
    }

    /**
     * Gets the current velocity of shooter motor 2.
     *
     * @return the velocity of shooter motor 2 in encoder ticks per second
     */
    public double getShooter2Velocity() {
        return shooterMotor2.getVelocity();
    }
}
