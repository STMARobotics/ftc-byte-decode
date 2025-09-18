package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    private final DcMotor turretMotor;
    private final Servo turretServo;

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}
