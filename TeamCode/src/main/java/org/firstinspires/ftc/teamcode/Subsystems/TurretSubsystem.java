package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MIN_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MIN_DEGREE;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Subsystem that controls the turret mechanism of the robot.
 * Uses a continuous rotation servo with a potentiometer for position feedback.
 */
public class TurretSubsystem extends SubsystemBase {

    private final CRServo turretServo;
    private final AnalogInput potentiometer;
    private final Telemetry telemetry;

    /**
     * Constructs a new TurretSubsystem.
     *
     * @param hardwareMap the hardware map from the OpMode
     * @param telemetry   the telemetry object for logging data
     */
    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretServo = hardwareMap.get(CRServo.class, "turretServo"); // port 3
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    /**
     * Sets the power of the turret servo.
     *
     * @param power the power to set, from -1.0 to 1.0
     */
    public void setTurretPower(double power) {
        turretServo.setPower(power);
    }

    /**
     * Gets the current position of the turret in degrees.
     * Uses the potentiometer voltage to calculate the position.
     *
     * @return the turret position in degrees within the configured min/max range
     */
    public double getTurretPosition() {
        double voltage = Math.max(POTENTIOMETER_MIN_VOLTAGE, Math.min(potentiometer.getVoltage(), POTENTIOMETER_MAX_VOLTAGE));
        double fraction = (voltage - POTENTIOMETER_MIN_VOLTAGE) / (POTENTIOMETER_MAX_VOLTAGE - POTENTIOMETER_MIN_VOLTAGE);
        return fraction * (TURRET_MAX_DEGREE - TURRET_MIN_DEGREE) + TURRET_MIN_DEGREE;
    }

    /**
     * Stops the turret servo.
     */
    public void stopTurret() {
        turretServo.setPower(0);
    }

    /**
     * Periodically called method for updating turret state.
     */
    public void periodic() {
        double voltage = Math.max(POTENTIOMETER_MIN_VOLTAGE, Math.min(potentiometer.getVoltage(), POTENTIOMETER_MAX_VOLTAGE));
        double fraction = (voltage - POTENTIOMETER_MIN_VOLTAGE) / (POTENTIOMETER_MAX_VOLTAGE - POTENTIOMETER_MIN_VOLTAGE);
    }

}
