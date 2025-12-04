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

public class IndexerSubsystem extends SubsystemBase {

    private final CRServo wheelIndexerServo;
    private final CRServo beltIndexerServo;
    private final ColorRangeSensor wheelDistanceSensor;
    private final ColorRangeSensor beltDistanceSensor;
    private final Telemetry telemetry;

    public IndexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        wheelIndexerServo = hardwareMap.get(CRServo.class, WHEEL_INDEXER_MOTOR_NAME);
        beltIndexerServo = hardwareMap.get(CRServo.class, BELT_INDEXER_MOTOR_NAME);
        beltIndexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelDistanceSensor = hardwareMap.get(ColorRangeSensor.class, WHEEL_SENSOR_NAME);
        beltDistanceSensor = hardwareMap.get(ColorRangeSensor.class, BELT_SENSOR_NAME);
    }

    public boolean isWheelSensorTripped() {
        return (wheelDistanceSensor.getDistance(DistanceUnit.INCH) < WHEEL_SENSOR_DISTANCE);
    }

    public boolean isBeltSensorTripped() {
        return (beltDistanceSensor.getDistance(DistanceUnit.INCH) < BELT_SENSOR_DISTANCE);
    }

    public void runBelt() {
        beltIndexerServo.setPower(BELT_INDEXING_SPEED);
    }

    public void runWheel() {
        wheelIndexerServo.setPower(1);
    }

    public void shoot() {
        wheelIndexerServo.setPower(WHEEL_INDEXING_SPEED);
    }

    public void stopWheel() {
        wheelIndexerServo.setPower(0);
    }

    public void stopBelt() {
        beltIndexerServo.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("belt distance sensor", beltDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("wheel distance sensor", wheelDistanceSensor.getDistance(DistanceUnit.INCH));
    }
}
