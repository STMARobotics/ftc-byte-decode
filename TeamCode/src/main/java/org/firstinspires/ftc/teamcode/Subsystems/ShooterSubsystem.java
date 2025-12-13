package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;

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

    public void startShooting(double speed) {
        shooterMotor1.setVelocity(speed);
        shooterMotor2.setVelocity(speed);
    }

    public void stopShooter() {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
    }

    public boolean isReadyToShoot(double speed) {
        return (Math.abs(Math.abs(shooterMotor1.getVelocity()) - speed) / 2 < 75);
    }

    public double getShooter1Velocity() {
        return shooterMotor1.getVelocity();
    }

    public double getShooter2Velocity() {
        return shooterMotor2.getVelocity();
    }
}
