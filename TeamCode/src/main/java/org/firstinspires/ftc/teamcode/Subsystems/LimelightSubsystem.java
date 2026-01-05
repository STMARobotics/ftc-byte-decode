package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Objects;

/**
 * Subsystem that interfaces with the Limelight 3A vision camera.
 * Used for detecting game elements and providing targeting information.
 */
public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    /**
     * Constructs a new LimelightSubsystem.
     *
     * @param hardwareMap the hardware map from the OpMode
     * @param telemetry   the telemetry object for logging data
     * @param team        the team color ("blue" or "red") to select the appropriate pipeline
     */
    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String team) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (Objects.equals(team, "blue")) {
            limelight.pipelineSwitch(0);
        } else {
            limelight.pipelineSwitch(1);
        }

        this.telemetry = telemetry;
    }

    /**
     * Starts the Limelight camera polling.
     */
    public void startLimelight() {
        limelight.start();
    }

    /**
     * Gets the latest result from the Limelight camera.
     *
     * @return the latest LLResult from the camera
     */
    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    /**
     * Checks if the Limelight has a valid target in view.
     *
     * @return true if a valid target is detected, false otherwise
     */
    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    /**
     * Gets the vertical offset (ty) to the target, which can be used for distance calculations.
     *
     * @return the ty value from the Limelight, or Double.NaN if no valid target
     */
    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Double.NaN;
        }
        return result.getTy();
    }

    /**
     * Periodically updates telemetry with Limelight data.
     */
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("Has Target", (result != null && result.isValid()));
        telemetry.addData("ty", limelight.getLatestResult().getTy());
    }
}