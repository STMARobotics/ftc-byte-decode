package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.INTERPOLATOR;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_SPEED;
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

public class ShootCommand extends CommandBase {

    private final IndexerSubsystem indexerSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final Telemetry telemetry;

    public ShootCommand(IndexerSubsystem indexerSubsystem,
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
    double tx = 0;

    @Override
    public void initialize() {
        shootingState = ShootingState.PREPARE;
        timer.reset();
        pid.setTolerance(TURRET_DEGREE_TOLERANCE);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.isBeltSensorTripped() && indexerSubsystem.isWheelSensorTripped()) {
            indexerSubsystem.stopBelt();
        } else {
            indexerSubsystem.runBelt();
        }

        double distanceToTarget = limelightSubsystem.getDistance();

        LookupTableMath.ShootingSettings s = INTERPOLATOR.calculate(distanceToTarget);

        double lookupRPM = s.getVelocity();

        telemetry.addData("velocity1", shooterSubsystem.getShooter1Velocity());
        telemetry.addData("velocity2", shooterSubsystem.getShooter2Velocity());
        telemetry.addData("target", lookupRPM);
        telemetry.addData("tx", tx);
        telemetry.addData("isReadyToShoot", shooterSubsystem.isReadyToShoot(lookupRPM));
        telemetry.addData("isInTolerance", Math.abs(tx) <= TURRET_DEGREE_TOLERANCE);
        telemetry.addData("number", Math.abs(Math.abs(shooterSubsystem.getShooter1Velocity()) - SHOOTING_SPEED));
        if (!limelightSubsystem.hasValidTarget()) {
            turretSubsystem.stopTurret();
        } else {
            LLResult result = limelightSubsystem.getLatestResult();
            tx = result.getTx();

            double pidOutput = pid.calculate(tx, 0);

            turretSubsystem.setTurretPower(pidOutput);
            telemetry.addData("pidoutput", pidOutput);
        }

        switch (shootingState) {
            case PREPARE:
                shooterSubsystem.startShooting(lookupRPM);
                if (shooterSubsystem.isReadyToShoot(lookupRPM) && Math.abs(tx) <= TURRET_DEGREE_TOLERANCE) {
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
        shooterSubsystem.stopShooter();
        indexerSubsystem.stopBelt();
    }
}