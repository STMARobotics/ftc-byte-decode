package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ShootingConstants;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShootingSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor;
    private final DcMotorEx indexerMotor;
    private final Telemetry telemetry;

    public ShootingSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        shooterMotor = hardwareMap.get(DcMotorEx.class, ShootingConstants.SHOOTER_MOTOR_NAME);
        indexerMotor = hardwareMap.get(DcMotorEx.class, ShootingConstants.INDEXER_MOTOR_NAME);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        indexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void runShooterMotor() {
        shooterMotor.setVelocity(ShootingConstants.SHOOTING_SPEED);
    }

    public void runShooterMotorAuto() {
        shooterMotor.setVelocity(1150);
    }

    public void runIndexer() {
        indexerMotor.setPower(ShootingConstants.INDEXING_SPEED);
    }

    public void stopIndexer() {
        indexerMotor.setPower(0);
    }

    public void stopShooter() {
        shooterMotor.setPower(0);
    }

    public double getShooterSpeed() {
        return shooterMotor.getVelocity();
    }

    @Override
    public void periodic() {
        telemetry.addData("Indexer Motor", indexerMotor.getPower());
        telemetry.addData("Shooter Speed", getShooterSpeed());
    }
}
