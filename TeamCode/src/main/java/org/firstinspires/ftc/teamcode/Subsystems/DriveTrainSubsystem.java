package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.Constants.SensorConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveTrainSubsystem extends SubsystemBase {

    private final DcMotor frontRight;
    private final DcMotor rearRight;
    private final DcMotor rearLeft;
    private final DcMotor frontLeft;
    private final SparkFunOTOS Otos;
    private final Follower follower;
    private final Telemetry telemetry;

    public DriveTrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);

        frontRight = hardwareMap.get(DcMotor.class, DriveTrainConstants.FRONT_RIGHT_MOTOR_NAME);
        rearRight = hardwareMap.get(DcMotor.class, DriveTrainConstants.BACK_RIGHT_MOTOR_NAME);
        rearLeft = hardwareMap.get(DcMotor.class, DriveTrainConstants.BACK_LEFT_MOTOR_NAME);
        frontLeft = hardwareMap.get(DcMotor.class, DriveTrainConstants.FRONT_LEFT_MOTOR_NAME);



        Otos = hardwareMap.get(SparkFunOTOS.class, SensorConstants.SPARKFUN_OTOS_NAME);
        Otos.setLinearUnit(DistanceUnit.METER);
        Otos.setAngularUnit(AngleUnit.RADIANS);
        Otos.setOffset(SensorConstants.OTOS_OFFSET);
        Otos.setLinearScalar(SensorConstants.OTOS_LINEAR_SCALAR);
        Otos.setAngularScalar(SensorConstants.OTOS_ANGULAR_SCALAR);
        Otos.calibrateImu();
        Otos.resetTracking();

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void driveFieldRelative(double forward, double right, double rotate) {

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                Otos.getPosition().h);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public Follower getFollower() {
        return follower;
    }

    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        rearLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        rearRight.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void resetLocalization() {
        Pose resetPose = new Pose();
        follower.setStartingPose(resetPose);
        follower.setPose(resetPose);
    }

    public void stopDrivetrain() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("x", Otos.getPosition().x);
        telemetry.addData("y", Otos.getPosition().y);
        telemetry.addData("h", Otos.getPosition().h);
    }
}