package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class SensorSubsystem extends SubsystemBase {

    static SparkFunOTOS Otos;
    private final Limelight3A limelight;

    public SensorSubsystem(HardwareMap hardwareMap) {

        Otos = hardwareMap.get(SparkFunOTOS.class, "Odometry Device");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

    }

    public static SparkFunOTOS.Pose2D getPose2d() {

        return (Otos.getPosition());

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
}