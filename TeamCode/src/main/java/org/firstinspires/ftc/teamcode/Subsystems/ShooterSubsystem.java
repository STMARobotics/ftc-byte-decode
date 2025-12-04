package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_SPEED;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;

    public ShooterSubsystem(HardwareMap hardwareMap) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startShooting() {
        shooterMotor1.setVelocity(SHOOTING_SPEED);
        shooterMotor2.setVelocity(SHOOTING_SPEED);
    }

    public void stopShooter() {
        shooterMotor1.setVelocity(0);
        shooterMotor2.setVelocity(0);
    }

    public boolean isReadyToShoot() {
        return (Math.abs(Math.abs(shooterMotor1.getVelocity()) - SHOOTING_SPEED) < 300);
    }

    public double getShooter1Velocity() {
        return shooterMotor1.getVelocity();
    }

    public double getShooter2Velocity() {
        return shooterMotor2.getVelocity();
    }
}
