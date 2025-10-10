package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShootingSubsystem;

public class ShootCommand extends CommandBase {

    private final ShootingSubsystem shootingSubsystem;
    private int shootingState = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public ShootCommand(ShootingSubsystem shootingSubsystem) {
        this.shootingSubsystem = shootingSubsystem;
    }

    @Override
    public void initialize() {
        shootingState = 0;
    }

    @Override
    public void execute() {
        switch(shootingState) {
            case 0:
                shootingSubsystem.runShooterMotor(1);
                if (shootingSubsystem.getShooterSpeed() == 60) {
                    shootingState = 1;
                    timer.reset();
                }
            case 1:
                shootingSubsystem.runIndexer(1);
                if (timer.seconds() == 2) {
                    shootingState = 2;
                    timer.reset();
                }
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.seconds() == 2 && shootingState == 1);
    }

    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.stop();
    }
}
