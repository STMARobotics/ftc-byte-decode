package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_SENSOR_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_SENSOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_SPEED;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
//    private final DistanceSensor distanceSensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
//        distanceSensor = hardwareMap.get(DistanceSensor.class, INTAKE_SENSOR_NAME);
    }

    public void runIntakeMotor() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    public boolean isSensorTripped() {
//        return (distanceSensor.getDistance(DistanceUnit.INCH) < INTAKE_SENSOR_DISTANCE);
        return false;
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}
