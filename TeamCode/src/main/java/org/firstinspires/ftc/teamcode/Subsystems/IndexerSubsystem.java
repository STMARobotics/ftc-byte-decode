package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.BELT_INDEXING_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.BELT_INDEXER_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.BELT_SENSOR_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.BELT_SENSOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.WHEEL_INDEXER_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.WHEEL_INDEXING_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.WHEEL_SENSOR_DISTANCE;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.WHEEL_SENSOR_NAME;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Subsystem responsible for managing the indexer mechanism that handles game elements.
 * Controls both wheel and belt indexer servos along with their associated distance sensors.
 */
public class IndexerSubsystem extends SubsystemBase {

    private final CRServo wheelIndexerServo;
    private final CRServo beltIndexerServo;
    private final ColorRangeSensor wheelDistanceSensor;
    private final DistanceSensor beltDistanceSensor;
    private final Telemetry telemetry;

    /**
     * Constructs a new IndexerSubsystem.
     *
     * @param hardwareMap the hardware map to retrieve devices from
     * @param telemetry the telemetry instance for logging
     */
    public IndexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        wheelIndexerServo = hardwareMap.get(CRServo.class, WHEEL_INDEXER_MOTOR_NAME);
        beltIndexerServo = hardwareMap.get(CRServo.class, BELT_INDEXER_MOTOR_NAME);
        beltIndexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelDistanceSensor = hardwareMap.get(ColorRangeSensor.class, WHEEL_SENSOR_NAME);
        beltDistanceSensor = hardwareMap.get(DistanceSensor.class, BELT_SENSOR_NAME);
    }

    /**
     * Checks if the wheel distance sensor has detected a game element.
     *
     * @return true if the sensor distance is less than the threshold, false otherwise
     */
    public boolean isWheelSensorTripped() {
        return (wheelDistanceSensor.getDistance(DistanceUnit.INCH) < WHEEL_SENSOR_DISTANCE);
    }

    /**
     * Checks if the belt distance sensor has detected a game element.
     *
     * @return true if the sensor distance is less than the threshold, false otherwise
     */
    public boolean isBeltSensorTripped() {
        return (beltDistanceSensor.getDistance(DistanceUnit.INCH) < BELT_SENSOR_DISTANCE);
    }

    /**
     * Runs the belt indexer servo at the configured indexing speed.
     */
    public void runBelt() {
        beltIndexerServo.setPower(BELT_INDEXING_SPEED);
    }

    /**
     * Runs the belt indexer servo in reverse at the configured indexing speed.
     */
    public void reverseBelt() {
        beltIndexerServo.setPower(-BELT_INDEXING_SPEED);
    }

    /**
     * Runs the wheel indexer servo at the configured indexing speed.
     */
    public void runWheel() {
        wheelIndexerServo.setPower(WHEEL_INDEXING_SPEED);
    }

    /**
     * Runs the wheel indexer servo in reverse at the configured indexing speed.
     */
    public void reverseWheel() {
        wheelIndexerServo.setPower(-WHEEL_INDEXING_SPEED);
    }

    /**
     * Activates the shooting mechanism by running the wheel indexer servo.
     */
    public void shoot() {
        wheelIndexerServo.setPower(WHEEL_INDEXING_SPEED);
    }

    /**
     * Stops the wheel indexer servo by setting its power to zero.
     */
    public void stopWheel() {
        wheelIndexerServo.setPower(0);
    }

    /**
     * Stops the belt indexer servo by setting its power to zero.
     */
    public void stopBelt() {
        beltIndexerServo.setPower(0);
    }

    @Override
    public void periodic() {
    }
}
