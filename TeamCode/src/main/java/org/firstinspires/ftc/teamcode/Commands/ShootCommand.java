package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_TIME;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_DEGREE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KD;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KP;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MAX_DEGREE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_MIN_DEGREE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class ShootCommand extends CommandBase {

    private final IndexerSubsystem indexerSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final Telemetry telemetry;

    public ShootCommand(IndexerSubsystem indexerSubsystem,
                        LimelightSubsystem limelightSubsystem,
                        TurretSubsystem turretSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        Telemetry telemetry) {
        this.indexerSubsystem = indexerSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.telemetry = telemetry;

        addRequirements(indexerSubsystem, turretSubsystem, limelightSubsystem, shooterSubsystem);
    }

    enum ShootingState {
        PREPARE,
        SHOOT,
        END
    }

    private ShootingState shootingState;
    private ElapsedTime timer = new ElapsedTime();
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private final ProfiledPIDController pid = new ProfiledPIDController(TURRET_KP, 0.0, TURRET_KD, constraints);

    @Override
    public void initialize() {
        shootingState = ShootingState.PREPARE;
        timer.reset();
        pid.setTolerance(TURRET_DEGREE_TOLERANCE);
    }

    @Override
    public void execute() {
        double tx = 0;
        if (!limelightSubsystem.hasValidTarget()) {
            turretSubsystem.stopTurret();
        } else {
            LLResult result = limelightSubsystem.getLatestResult();
            tx = result.getTx();

            double pidOutput = pid.calculate(tx, 0);

            if (turretSubsystem.getTurretPosition() <= TURRET_MIN_DEGREE && pidOutput < 0 ||
                    turretSubsystem.getTurretPosition() >= TURRET_MAX_DEGREE && pidOutput > 0) {
                pidOutput = 0;
            }

            turretSubsystem.setTurretPower(pidOutput);
        }

        switch (shootingState) {
            case PREPARE:
                shooterSubsystem.startShooting();
                /*
                telemetry.addData("velocity1", shooterSubsystem.getShooter1Velocity());
                telemetry.addData("velocity2", shooterSubsystem.getShooter2Velocity());
                telemetry.addData("tx", tx);
                telemetry.addData("isReadyToShoot", shooterSubsystem.isReadyToShoot());
                telemetry.addData("isInTolerance", Math.abs(tx) <= TURRET_DEGREE_TOLERANCE);
                telemetry.addData("number", Math.abs(shooterSubsystem.getShooter1Velocity()) - SHOOTING_SPEED);
                telemetry.addData("number2", Math.abs(shooterSubsystem.getShooter2Velocity()) - SHOOTING_SPEED);
                 */
                if (shooterSubsystem.isReadyToShoot() && Math.abs(tx) <= TURRET_DEGREE_TOLERANCE) {
                    shootingState = ShootingState.SHOOT;
                }
                break;
            case SHOOT:
                indexerSubsystem.shoot();
                timer.reset();
                if (timer.seconds() > SHOOTING_TIME) {
                    shootingState = ShootingState.END;
                    timer.reset();
                }
                break;
        }
//        telemetry.addData("ShootingState", shootingState);

    }

    public boolean isFinished() {
        return (shootingState == ShootingState.END);
    }

    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
        indexerSubsystem.stopWheel();
    }
}