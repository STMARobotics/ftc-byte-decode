package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Constants.ShootingConstants;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShootingSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor;
    private final DcMotorEx indexerMotor;

    public ShootingSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, ShootingConstants.SHOOTER_MOTOR_NAME);
        indexerMotor = hardwareMap.get(DcMotorEx.class, ShootingConstants.INDEXER_MOTOR_NAME);
    }

    public void runShooterMotor(double speed) {
        shooterMotor.setPower(speed);
    }

    public void runIndexer(double speed) {
        indexerMotor.setPower(speed);
    }

    public void stop() {
        shooterMotor.setPower(0);
        indexerMotor.setPower(0);
    }

    public double getShooterSpeed() {
        return shooterMotor.getVelocity(AngleUnit.DEGREES);
    }


}
