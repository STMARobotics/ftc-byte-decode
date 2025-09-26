package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ManipulationSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
    private final DcMotorEx shootingMotor;
    private final CRServo indexerServo;

    public ManipulationSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");
        indexerServo = hardwareMap.get(CRServo.class, "indexerServo");

        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void runIndexer(double power) {
        indexerServo.setPower(power);
    }

    public void runShooter() {
        shootingMotor.setVelocity(100, AngleUnit.DEGREES);
    }

    public void stopIntake() {
        intakeMotor.setPower(0);
    }

    public void stopIndexer() {
        indexerServo.setPower(0);
    }

    public void stopShooter() {
        shootingMotor.setPower(0);
    }


    public double getShooterSpeed() {
        return shootingMotor.getVelocity(AngleUnit.DEGREES);
    }

}
