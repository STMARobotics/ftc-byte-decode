package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_TIME;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

public class ShootCommand extends CommandBase {

    private final IndexerSubsystem indexerSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final TurretSubsystem turretSubsystem;

    public ShootCommand(IndexerSubsystem indexerSubsystem,
                        LimelightSubsystem limelightSubsystem,
                        TurretSubsystem turretSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.turretSubsystem = turretSubsystem;
    }

    enum ShootingState {
        PREPARE,
        SHOOT,
        END
    }

    private ShootingState shootingState;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {
        shootingState = ShootingState.PREPARE;
        timer.reset();
    }

    public void execute() {
        LLResult result = limelightSubsystem.getLatestResult();
        switch (shootingState) {
            case PREPARE:
                turretSubsystem.shoot();
                if (turretSubsystem.isReadyToShoot()) {
                    shootingState = ShootingState.SHOOT;
                }
                break;
            case SHOOT:
                indexerSubsystem.index();
                timer.reset();
                if (timer.seconds() > SHOOTING_TIME) {
                    shootingState = ShootingState.END;
                    timer.reset();
                }
                break;
            case END:
            indexerSubsystem.stop();
            turretSubsystem.stop();
        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {

    }
}