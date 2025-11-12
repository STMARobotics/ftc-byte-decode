package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void startLimelight() {
        limelight.start();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public String getMotif(int tag) {
        switch(tag) {
            case 21:
                return "GPP";
            case 22:
                return "PGP";
            case 23:
                return "PPG";
        }
        return "None";
    }

    public void periodic() {
        telemetry.addData("Motif", sensorSubsystem.getMotif(newTagId));
    }
}