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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class AutoShootCommand extends CommandBase {

    private static final Logger log = LoggerFactory.getLogger(AutoShootCommand.class);
    private final IndexerSubsystem indexerSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final Telemetry telemetry;

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

    enum ShootingState {
        PREPARE,
        SHOOT,
        END
    }

    private ShootingState shootingState;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    private final ProfiledPIDController pid = new ProfiledPIDController(TURRET_KP, 0.0, TURRET_KD, constraints);
    double tx = 0;
    private double shot = 0;

    @Override
    public void initialize() {
        shootingState = ShootingState.PREPARE;
        timer.reset();
        timer2.reset();
        pid.setTolerance(TURRET_DEGREE_TOLERANCE);
        shot = 0;
    }

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

        telemetry.addData("isWheelTripped", indexerSubsystem.isWheelSensorTripped());
        telemetry.addData("isBeltTripped", indexerSubsystem.isBeltSensorTripped());
        telemetry.addData("velocity1", shooterSubsystem.getShooter1Velocity() /28*60);
        telemetry.addData("velocity2", shooterSubsystem.getShooter2Velocity() /28*60);
        telemetry.addData("target", lookupRPM);
        telemetry.addData("tx", tx);
        telemetry.addData("shot", shot);
        if (!limelightSubsystem.hasValidTarget()) {
            turretSubsystem.stopTurret();
        } else {
            try {
                LLResult result = limelightSubsystem.getLatestResult();
                tx = result.getTx();

                double pidOutput = pid.calculate(tx, 0);

                turretSubsystem.setTurretPower(pidOutput);
            } catch (NullPointerException error) {
                log.error("e: ", error);
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
                    shot += 1;
                }
                break;
        }
    }

    public boolean isFinished() {
        return (shot > 6);
    }

    public void end(boolean interrupted) {
        turretSubsystem.stopTurret();
        indexerSubsystem.stopWheel();
        shooterSubsystem.stopShooter();
        indexerSubsystem.stopBelt();
        intakeSubsystem.stop();
    }
}