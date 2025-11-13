package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
    }

    public void runIntakeMotor() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
