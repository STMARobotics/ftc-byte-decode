package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MIN_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MIN_DEGREE;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {

    private final CRServo turretServo;
    private final AnalogInput potentiometer;
    private final Telemetry telemetry;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretServo = hardwareMap.get(CRServo.class, "turretServo"); // port 3
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    public void setTurretPower(double power) {
        turretServo.setPower(power);
    }

    public double getTurretPosition() {
        double voltage = Math.max(POTENTIOMETER_MIN_VOLTAGE, Math.min(potentiometer.getVoltage(), POTENTIOMETER_MAX_VOLTAGE));
        double fraction = (voltage - POTENTIOMETER_MIN_VOLTAGE) / (POTENTIOMETER_MAX_VOLTAGE - POTENTIOMETER_MIN_VOLTAGE);
        return fraction * (TURRET_MAX_DEGREE - TURRET_MIN_DEGREE) + TURRET_MIN_DEGREE;
    }

    public void stopTurret() {
        turretServo.setPower(0);
    }

    public void periodic() {
        double voltage = Math.max(POTENTIOMETER_MIN_VOLTAGE, Math.min(potentiometer.getVoltage(), POTENTIOMETER_MAX_VOLTAGE));
        double fraction = (voltage - POTENTIOMETER_MIN_VOLTAGE) / (POTENTIOMETER_MAX_VOLTAGE - POTENTIOMETER_MIN_VOLTAGE);
        telemetry.addData("getCurrent", fraction * (TURRET_MAX_DEGREE - TURRET_MIN_DEGREE) + TURRET_MIN_DEGREE);
    }

}
