package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void runIntakeMotor() {
        intakeMotor.setPower(IntakeConstants.INTAKE_SPEED);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
