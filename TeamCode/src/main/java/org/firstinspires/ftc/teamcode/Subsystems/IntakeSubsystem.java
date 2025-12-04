package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_SENSOR_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_SENSOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IntakeConstants.INTAKE_SPEED;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
    private final ColorRangeSensor distanceSensor;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        distanceSensor = hardwareMap.get(ColorRangeSensor.class, INTAKE_SENSOR_NAME);
    }

    public void runIntakeMotor() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    public boolean isSensorTripped() {
        return (distanceSensor.getDistance(DistanceUnit.INCH) < INTAKE_SENSOR_DISTANCE);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}
