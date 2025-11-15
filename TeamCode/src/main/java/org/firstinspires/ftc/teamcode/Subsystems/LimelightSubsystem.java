package org.firstinspires.ftc.teamcode.Subsystems;
/*
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final int newTagId;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry, int newTagId) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        this.telemetry = telemetry;
        this.newTagId = newTagId;
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

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public void periodic() {
        telemetry.addData("Motif", getMotif(newTagId));
    }
}

 */