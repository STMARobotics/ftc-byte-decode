package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ManipulationSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorSubsystem;

public class AutoShootCommand extends CommandBase {

    private final DriveTrainSubsystem driveTrainSubsystem;
    private final ManipulationSubsystem manipulationSubsystem;
    private final SensorSubsystem sensorSubsystem;
    private final double multiplicationConstant = 0.1;
    private int shootingState;

    public AutoShootCommand(
            DriveTrainSubsystem driveTrainSubsystem,
            ManipulationSubsystem manipulationSubsystem,
            SensorSubsystem sensorSubsystem) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.manipulationSubsystem = manipulationSubsystem;
        this.sensorSubsystem = sensorSubsystem;
    }

    public void initialize() {
        shootingState = 0;
    }

    public void execute() {
        LLResult results = sensorSubsystem.getLatestResult();
        switch (shootingState) {
            case 0:
                double tx = (results.getTx())*multiplicationConstant;
                driveTrainSubsystem
                        .drive(0, 0, tx);
                manipulationSubsystem.runShooter();
                if (manipulationSubsystem.getShooterSpeed() >= 30 && Math.abs(tx) <= 1) {
                    shootingState += 1;
                    break;
                }
            case 1:
        }



    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {

    }
}
