package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Constants.ShootingConstants;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShootingSubsystem;

public class ShootCommand extends CommandBase {

    private final ShootingSubsystem shootingSubsystem;
    private int shootingState = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public ShootCommand(ShootingSubsystem shootingSubsystem) {
        this.shootingSubsystem = shootingSubsystem;

        addRequirements(shootingSubsystem);
    }

    @Override
    public void initialize() {
        shootingState = 0;
    }

    @Override
    public void execute() {
        switch(shootingState) {
            case 0:
                shootingSubsystem.runShooterMotor();
                if (shootingSubsystem.getShooterSpeed() >= ShootingConstants.SHOOTING_SPEED) {
                    shootingState = 1;
                    timer.reset();
                }
                break;
            case 1:
                shootingSubsystem.runIndexer();
                if (timer.seconds() >= 2) {
                    shootingState = 2;
                    timer.reset();
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.seconds() >= 2 && shootingState == 2);
    }

    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.stopIndexer();
        shootingSubsystem.stopShooter();
    }
}