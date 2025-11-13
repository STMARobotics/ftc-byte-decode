package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MAX_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.POTENTIOMETER_MIN_VOLTAGE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_DEGREE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KD;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KP;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MIN_DEGREE;

public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor1;
    private final DcMotorEx turretMotor2;
    private final CRServo turretServo;
    private final AnalogInput potentiometer;
    private final LimelightSubsystem limelightSubsystem;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    private final ProfiledPIDController pid = new ProfiledPIDController(TURRET_KP, 0.0, TURRET_KD, constraints);

    public TurretSubsystem(HardwareMap hardwareMap, LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;

        turretMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        turretMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        pid.setTolerance(TURRET_DEGREE_TOLERANCE);

    }

    public void setTurretAngle(double angle) {

    }

    public void lockTurret() {
    }

    public void autoAimTurret() {
        if (!limelightSubsystem.hasValidTarget()) {
            stopTurret();
            return;
        }

        LLResult result = limelightSubsystem.getLatestResult();
        double tx = result.getTx();

        if (Math.abs(tx) <= TURRET_DEGREE_TOLERANCE) {
            stopTurret();
            return;
        }

        double pidOutput = pid.calculate(tx, 0);
        
        if (getTurretPosition() <= TURRET_MIN_DEGREE && pidOutput < 0 ||
            getTurretPosition() >= TURRET_MAX_DEGREE && pidOutput > 0) {
            pidOutput = 0;
        }

        turretServo.setPower(pidOutput);
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
