package org.firstinspires.ftc.teamcode.OPmodes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ForkliftSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.StagingSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

@Autonomous
public class AutoOPMode extends LinearOpMode {

    private DriveTrainSubsystem driveTrainSubsystem;
    private SensorSubsystem sensorSubsystem;
    private ForkliftSubsystem forkliftSubsystem;
    private IndexerSubsystem indexerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private StagingSubsystem stagingSubsystem;
    private TurretSubsystem turretSubsystem;

    @Override
    public void runOpMode() {

        driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap);
        sensorSubsystem = new SensorSubsystem(hardwareMap);
        forkliftSubsystem = new ForkliftSubsystem(hardwareMap);
        indexerSubsystem = new IndexerSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        stagingSubsystem = new StagingSubsystem(hardwareMap);
        turretSubsystem = new TurretSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            SparkFunOTOS.Pose2D pose = SensorSubsystem.getPose2d();

            telemetry.addData("Status", "Running");
            telemetry.update();
            telemetry.addData("X coordinate", pose.x);
            telemetry.addData("Y coordinate", pose.y);
            telemetry.addData("Heading angle", pose.h);

        }
    }
}
