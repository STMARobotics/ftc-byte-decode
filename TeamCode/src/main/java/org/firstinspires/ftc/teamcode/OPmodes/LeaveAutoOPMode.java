package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Commands.LeaveAutoCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;

@Autonomous
public class LeaveAutoOPMode extends CommandOpMode {

    @Override
    public void initialize() {
        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);

        register(driveTrainSubsystem);

        schedule(new LeaveAutoCommand(driveTrainSubsystem));

    }
}
