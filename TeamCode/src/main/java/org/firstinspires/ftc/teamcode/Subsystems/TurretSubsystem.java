package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    private final DcMotor turretMotor;

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}
