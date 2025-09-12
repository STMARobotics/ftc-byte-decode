package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SensorSubsystem {

    static SparkFunOTOS Otos;

    public SensorSubsystem(HardwareMap hardwareMap) {
        Otos = hardwareMap.get(SparkFunOTOS.class, "Odometry Device");
    }

    public static SparkFunOTOS.Pose2D getPose2d() {
        return (Otos.getPosition());
    }
}
