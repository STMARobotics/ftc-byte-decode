package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Objects;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String team) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (Objects.equals(team, "blue")) {
            limelight.pipelineSwitch(0);
        } else {
            limelight.pipelineSwitch(1);
        }

        this.telemetry = telemetry;
    }

    public void startLimelight() {
        limelight.start();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Double.NaN;
        }
        return result.getTy();
    }

    public void periodic() {
        LLResult result = limelight.getLatestResult();
//        telemetry.addData("Has Target", (result != null && result.isValid()));
        telemetry.addData("ty", limelight.getLatestResult().getTy());
    }
}