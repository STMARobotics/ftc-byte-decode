package org.firstinspires.ftc.teamcode.OPmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;

import org.firstinspires.ftc.teamcode.Commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem;

import java.util.List;

@Autonomous
public class RGAutoOPMode extends CommandOpMode {

    private int newTagId = 0;

    @Override
    public void initialize() {
        int lastTagId = 0;

        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(hardwareMap, telemetry);
 //       LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry, newTagId);
        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);
        IndexerSubsystem indexerSubsystem = new IndexerSubsystem(hardwareMap);
        TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap);

        /*      limelightSubsystem.startLimelight();
        LLResult result = limelightSubsystem.getLatestResult();
        List<LLResultTypes.FiducialResult> apriltagResult = result.getFiducialResults();

        newTagId = lastTagId;
        for (LLResultTypes.FiducialResult detection : apriltagResult) {
            if (detection.getFiducialId() >= 21 && detection.getFiducialId() <= 23) {
                newTagId = detection.getFiducialId();
                break;
            }
        }


         */
        Pose startPose = new Pose(60, 9.000, Math.toRadians(90));
        PathChain path = driveTrainSubsystem.pathBuilder()
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(60.000, 9.000), new Pose(42.829, 34.962))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(42.829, 34.962), new Pose(18.792, 34.962))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(18.792, 34.962), new Pose(71.454, 18.355))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .addPath(
                        // Path 4
                        new BezierLine(new Pose(71.454, 18.355), new Pose(57.906, 60.310))
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(57.906, 60.310), new Pose(24.036, 60.091))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(24.036, 60.091), new Pose(58.780, 93.742))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(58.780, 93.742), new Pose(38.021, 84.346))
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(38.021, 84.346), new Pose(19.448, 84.127))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(19.448, 84.127), new Pose(43.047, 109.256))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

        register(driveTrainSubsystem,
         //       limelightSubsystem,
                indexerSubsystem,
                intakeSubsystem,
                turretSubsystem);

        FollowPathCommand followPathCommand =
                new FollowPathCommand(startPose, path, driveTrainSubsystem)
                        .withGlobalMaxPower(0.5);


        RunCommand intakeCommand =
                new RunCommand(intakeSubsystem::runIntakeMotor);

        RunCommand stopIntakeCommand =
                new RunCommand(intakeSubsystem::stop);

        new RunCommand(() ->
                telemetry.addData("isScheduled", followPathCommand.isScheduled())).schedule();
        new RunCommand(telemetry::update).schedule();
    }

}
