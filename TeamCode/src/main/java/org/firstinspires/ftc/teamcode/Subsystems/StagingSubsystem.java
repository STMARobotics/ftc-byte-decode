package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class StagingSubsystem extends SubsystemBase {

    private final Servo stagingMotor1;
    private final Servo stagingMotor2;

    public StagingSubsystem(HardwareMap hardwareMap) {

        stagingMotor1 = hardwareMap.get(Servo.class, "stagingMotor1");
        stagingMotor2 = hardwareMap.get(Servo.class, "stagingMotor2");

    }
}
