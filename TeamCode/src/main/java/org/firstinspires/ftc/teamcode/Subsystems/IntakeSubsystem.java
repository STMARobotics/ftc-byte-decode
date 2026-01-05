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

/**
 * Subsystem that controls the intake mechanism of the robot.
 * Handles the intake motor and color/range sensor for detecting game elements.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final DcMotor intakeMotor;
    private final ColorRangeSensor distanceSensor;
    private final Telemetry telemetry;

    /**
     * Constructs a new IntakeSubsystem.
     *
     * @param hardwareMap the hardware map from the OpMode
     * @param telemetry   the telemetry object for logging data
     */
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        distanceSensor = hardwareMap.get(ColorRangeSensor.class, INTAKE_SENSOR_NAME);
    }

    /**
     * Runs the intake motor at the configured intake speed.
     */
    public void runIntakeMotor() {
        intakeMotor.setPower(INTAKE_SPEED);
    }

    /**
     * Runs the intake motor in reverse at the configured intake speed.
     */
    public void reverse() {
        intakeMotor.setPower(-INTAKE_SPEED);
    }

    /**
     * Checks if the distance sensor detects an object within the threshold distance.
     *
     * @return true if an object is detected within INTAKE_SENSOR_DISTANCE, false otherwise
     */
    public boolean isSensorTripped() {
        return (distanceSensor.getDistance(DistanceUnit.INCH) < INTAKE_SENSOR_DISTANCE);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }

    @Override
    public void periodic() {
//        telemetry.addData("Intake Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}
