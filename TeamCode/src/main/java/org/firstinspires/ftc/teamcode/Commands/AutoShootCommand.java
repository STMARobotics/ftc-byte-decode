package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.INTERPOLATOR;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_TIME;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_DEGREE_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KD;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KP;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.ProfiledPIDController;
import com.seattlesolvers.solverslib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.LookupTableMath;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

/**
 * Command that autonomously aims and shoots at a target using vision tracking.
 * <p>
 * This command coordinates multiple subsystems to:
 * <ul>
 *   <li>Track targets using the Limelight vision system</li>
 *   <li>Aim the turret using a profiled PID controller</li>
 *   <li>Spin up the shooter to the appropriate velocity based on distance</li>
 *   <li>Manage the indexer and intake to feed balls into the shooter</li>
 * </ul>
 * The command runs for 8 seconds before automatically finishing.
 */
public class AutoShootCommand extends CommandBase {

    private final IndexerSubsystem indexerSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final Telemetry telemetry;

    /**
     * Constructs an AutoShootCommand with all required subsystems.
     *
     * @param indexerSubsystem   the indexer subsystem for feeding balls
     * @param limelightSubsystem the limelight subsystem for vision tracking
     * @param turretSubsystem    the turret subsystem for aiming
     * @param shooterSubsystem   the shooter subsystem for launching balls
     * @param intakeSubsystem    the intake subsystem for collecting balls
     * @param telemetry          the telemetry object for logging data
     */
    public AutoShootCommand(IndexerSubsystem indexerSubsystem,
                            LimelightSubsystem limelightSubsystem,
                            TurretSubsystem turretSubsystem,
                            ShooterSubsystem shooterSubsystem,
                            IntakeSubsystem intakeSubsystem,
                            Telemetry telemetry) {
        this.indexerSubsystem = indexerSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.telemetry = telemetry;

        addRequirements(indexerSubsystem, turretSubsystem, limelightSubsystem, shooterSubsystem, intakeSubsystem);
    }

    /**
     * Represents the current state of the shooting sequence.
     */
    enum ShootingState {
        /** Preparing to shoot - spinning up flywheel and aiming */
        PREPARE,
        /** Actively shooting - feeding balls into the shooter */
        SHOOT,
    }

    private ShootingState shootingState;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private final ProfiledPIDController pid = new ProfiledPIDController(TURRET_KP, 0.0, TURRET_KD, constraints);
    double tx = 0;

    /**
     * Initializes the command by resetting timers and setting the initial state.
     */
    @Override
    public void initialize() {
        shootingState = ShootingState.PREPARE;
        timer.reset();
        timer2.reset();
        pid.setTolerance(TURRET_DEGREE_TOLERANCE);
    }

    /**
     * Executes the shooting sequence by:
     * <ol>
     *   <li>Managing the indexer belt based on sensor states</li>
     *   <li>Controlling the intake based on ball position sensors</li>
     *   <li>Calculating shooter velocity from distance using lookup table</li>
     *   <li>Aiming the turret using PID control based on Limelight data</li>
     *   <li>Transitioning between PREPARE and SHOOT states when ready</li>
     * </ol>
     */
    @Override
    public void execute() {
        if (indexerSubsystem.isBeltSensorTripped() && indexerSubsystem.isWheelSensorTripped()) {
            indexerSubsystem.stopBelt();
        } else {
            indexerSubsystem.runBelt();
        }

        if (intakeSubsystem.isSensorTripped() && indexerSubsystem.isWheelSensorTripped() && indexerSubsystem.isBeltSensorTripped()) {
            intakeSubsystem.stop();
        } else {
            intakeSubsystem.runIntakeMotor();
        }

        double distanceToTarget = limelightSubsystem.getDistance();

        LookupTableMath.ShootingSettings s = INTERPOLATOR.calculate(distanceToTarget);

        double lookupRPM = s.getVelocity();

        telemetry.addData("velocity1", shooterSubsystem.getShooter1Velocity() /28*60);
        telemetry.addData("velocity2", shooterSubsystem.getShooter2Velocity() /28*60);
        telemetry.addData("target", lookupRPM);
        telemetry.addData("isReadytoShoot", shooterSubsystem.isReadyToShoot(lookupRPM));
        telemetry.addData("tolernace?", Math.abs(tx) <= TURRET_DEGREE_TOLERANCE);
        telemetry.addData("tx", tx);
        if (!limelightSubsystem.hasValidTarget()) {
            turretSubsystem.stopTurret();
        } else {
            try {
                LLResult result = limelightSubsystem.getLatestResult();
                tx = result.getTx();

                double pidOutput = pid.calculate(tx, 0);

                turretSubsystem.setTurretPower(pidOutput);
            } catch (NullPointerException error) {
            }
        }

        switch (shootingState) {
            case PREPARE:
                shooterSubsystem.startShooting(lookupRPM);
                if (shooterSubsystem.isReadyToShoot(lookupRPM) && Math.abs(tx) <= TURRET_DEGREE_TOLERANCE && timer.seconds() > SHOOTING_TIME) {
                    shootingState = ShootingState.SHOOT;
                    timer.reset();
                }
                break;
            case SHOOT:
                indexerSubsystem.shoot();
                if (timer.seconds() > SHOOTING_TIME) {
                    shootingState = ShootingState.PREPARE;
                    timer.reset();
                }
                break;
        }
    }

    /**
     * Determines if the command has finished executing.
     *
     * @return {@code true} if the command has run for more than 8 seconds, {@code false} otherwise
     */
    public boolean isFinished() {
        return (timer2.seconds() > 8);
    }

    /**
     * Stops all subsystems when the command ends.
     *
     * @param interrupted {@code true} if the command was interrupted, {@code false} if it ended normally
     */
    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
        indexerSubsystem.stopWheel();
        shooterSubsystem.stopShooter();
        indexerSubsystem.stopBelt();
        intakeSubsystem.stop();
    }
}