package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ForkliftSubsystem extends SubsystemBase {

    private final DcMotor forkliftMotor;

    public ForkliftSubsystem(HardwareMap hardwareMap) {

        forkliftMotor = hardwareMap.get(DcMotor.class, "forkliftMotor");
        forkliftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public int getMotorPosition() {
        return forkliftMotor.getCurrentPosition();
    }

    public void setMotorPosition(int position) {
        forkliftMotor.setTargetPosition(position);
    }
}
