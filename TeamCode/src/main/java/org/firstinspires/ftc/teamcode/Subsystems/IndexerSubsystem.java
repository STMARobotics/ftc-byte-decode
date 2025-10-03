package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private final CRServo indexerLeftServo;
    private final CRServo indexerRightServo;

    public IndexerSubsystem(HardwareMap hardwareMap) {
        indexerLeftServo = hardwareMap.get(CRServo.class, "Indexer Left Servo");
        indexerRightServo = hardwareMap.get(CRServo.class, "Indexer Right Servo");
    }

    public void runLeftIndexer(double power) {
        indexerLeftServo.setPower(power);
    }

    public void runRightIndexer(double power) {
        indexerRightServo.setPower(power);
    }

    public void stop() {
        indexerRightServo.setPower(0);
        indexerLeftServo.setPower(0);
    }
}
