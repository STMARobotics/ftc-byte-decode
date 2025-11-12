package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.INDEXING_SPEED;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.LEFT_INDEXER_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.Constants.IndexerConstants.RIGHT_INDEXER_MOTOR_NAME;

public class IndexerSubsystem extends SubsystemBase {

    private final CRServo indexerLeftServo;
    private final CRServo indexerRightServo;

    public IndexerSubsystem(HardwareMap hardwareMap) {
        indexerLeftServo = hardwareMap.get(CRServo.class, LEFT_INDEXER_MOTOR_NAME);
        indexerRightServo = hardwareMap.get(CRServo.class, RIGHT_INDEXER_MOTOR_NAME);
    }

    public void runLeftIndexer(double power) {
        indexerLeftServo.setPower(power);
    }

    public void runRightIndexer(double power) {
        indexerRightServo.setPower(power);
    }

    public void index() {
        runLeftIndexer(INDEXING_SPEED);
        runRightIndexer(INDEXING_SPEED);
    }

    public void stop() {
        indexerRightServo.setPower(0);
        indexerLeftServo.setPower(0);
    }
}
