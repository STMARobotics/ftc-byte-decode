package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    private final DcMotor turretMotor;
    private final CRServo turretServo;

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setTurretAngle(double angle) {

    }

}
