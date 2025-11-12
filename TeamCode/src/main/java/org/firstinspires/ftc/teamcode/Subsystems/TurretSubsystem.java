package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.SHOOTING_SPEED;

public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor1;
    private final DcMotorEx turretMotor2;
    private final CRServo turretServo;

    public TurretSubsystem(HardwareMap hardwareMap) {

        turretMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        turretMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");

    }

    public void setTurretAngle(double angle) {

    }

    public void shoot() {
        turretMotor1.setVelocity(SHOOTING_SPEED);
        turretMotor2.setVelocity(SHOOTING_SPEED);
    }

    public boolean isReadyToShoot() {
        return (Math.abs(turretMotor1.getVelocity() - SHOOTING_SPEED) < 200 &&
                Math.abs(turretMotor2.getVelocity() - SHOOTING_SPEED) < 200);
    }

    public void stop() {
        turretMotor1.setVelocity(0);
        turretMotor2.setVelocity(0);
    }

}
