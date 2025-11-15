package org.firstinspires.ftc.teamcode.Commands;

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

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class ShootCommand extends CommandBase {

    private final IndexerSubsystem indexerSubsystem;
 //   private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;

    public ShootCommand(IndexerSubsystem indexerSubsystem,
 //                       LimelightSubsystem limelightSubsystem,
                        TurretSubsystem turretSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
 //       this.limelightSubsystem = limelightSubsystem;
        this.turretSubsystem = turretSubsystem;

        addRequirements(indexerSubsystem, turretSubsystem);
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

    public void execute() {
  /*      if (!limelightSubsystem.hasValidTarget()) {
            turretSubsystem.stopTurret();
        }

        LLResult result = limelightSubsystem.getLatestResult();
        double tx = result.getTx();


   */
        double pidOutput = pid.calculate(1, 0);

        if (turretSubsystem.getTurretPosition() <= TURRET_MIN_DEGREE && pidOutput < 0 ||
                turretSubsystem.getTurretPosition() >= TURRET_MAX_DEGREE && pidOutput > 0) {
            pidOutput = 0;
        }

        turretSubsystem.setTurretPower(pidOutput);

        switch (shootingState) {
            case PREPARE:
                turretSubsystem.shoot();
                if (turretSubsystem.isReadyToShoot() && Math.abs(1) <= TURRET_DEGREE_TOLERANCE) {
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
    }

    public boolean isFinished() {
        return (shootingState == ShootingState.END);
    }

    public void end(boolean interrupted) {
        turretSubsystem.stopShooter();
        turretSubsystem.stopTurret();
    }
}