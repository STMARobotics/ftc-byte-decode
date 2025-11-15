package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MIN_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MIN_DEGREE;

public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor1;
    private final DcMotorEx turretMotor2;
    private final CRServo turretServo;
    private final AnalogInput potentiometer;

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        turretMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
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

    public void shoot() {
        turretMotor1.setVelocity(SHOOTING_SPEED);
        turretMotor2.setVelocity(SHOOTING_SPEED);
    }

    public boolean isReadyToShoot() {
        return (Math.abs(turretMotor1.getVelocity() - SHOOTING_SPEED) < 200 &&
                Math.abs(turretMotor2.getVelocity() - SHOOTING_SPEED) < 200);
    }

    public void stopShooter() {
        turretMotor1.setVelocity(0);
        turretMotor2.setVelocity(0);
    }

    public void stopTurret() {
        turretServo.setPower(0);
    }

}
