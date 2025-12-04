package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_DEGREE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KD;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KP;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MIN_DEGREE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class LockTurretCommand extends CommandBase {

    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private final ProfiledPIDController pid = new ProfiledPIDController(TURRET_KP, 0.0, TURRET_KD, constraints);
    private final Telemetry telemetry;

    public LockTurretCommand(LimelightSubsystem limelightSubsystem, TurretSubsystem turretSubsystem, Telemetry telemetry) {
        this.limelightSubsystem = limelightSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.telemetry = telemetry;
    }

    public void initialize() {
        pid.setTolerance(TURRET_DEGREE_TOLERANCE);
    }

    public void execute() {
        if (!limelightSubsystem.hasValidTarget()) {
            turretSubsystem.stopTurret();
        }

        LLResult result = limelightSubsystem.getLatestResult();
        double tx = result.getTx();

        double pidOutput = pid.calculate(tx, 0);

        if (turretSubsystem.getTurretPosition() >= TURRET_MIN_DEGREE && pidOutput < 0 ||
                turretSubsystem.getTurretPosition() <= TURRET_MAX_DEGREE && pidOutput > 0) {
            turretSubsystem.setTurretPower(pidOutput);
        }
    }

}
