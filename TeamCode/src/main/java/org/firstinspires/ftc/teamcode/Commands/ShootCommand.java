package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;

public class ShootCommand extends CommandBase {

    private final IndexerSubsystem indexerSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    public ShootCommand(IndexerSubsystem indexerSubsystem,
                             LimelightSubsystem limelightSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.limelightSubsystem = limelightSubsystem;
    }

    enum ShootingState {
        IDLE,
        PREPARE,
        SHOOT,
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
                indexerSubsystem.runLeftIndexer(1);



        }
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {

    }
}
/*
boolean launch(boolean shotRequested){
    switch (launchState) {
        case IDLE:
            if (shotRequested) {
                launchState = LaunchState.PREPARE;
                shotTimer.reset();
            }
            break;
        case PREPARE:
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY){
                launchState = LaunchState.LAUNCH;
                leftFeeder.setPower(1);
                rightFeeder.setPower(1);
                feederTimer.reset();
            }
            break;
        case LAUNCH:
            if (feederTimer.seconds() > FEED_TIME) {
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);

                if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                    launchState = LaunchState.IDLE;
                    return true;
                }
            }
    }
    return false;
}

*/