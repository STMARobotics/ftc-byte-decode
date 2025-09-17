package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private final Servo indexerServo;

    public IndexerSubsystem(HardwareMap hardwareMap) {

        indexerServo = hardwareMap.get(Servo.class, "Indexer Servo");

    }
}
