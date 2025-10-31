package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.core.math.MathUtils;

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

    private final Follower follower;
    private final Telemetry telemetry;
    private Pose currentPose = new Pose();

    public DriveTrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);
    }

    public void drive(double translationX, double translationY, double rotation, double reductionFactor) {
        double clampedReduction = MathUtils.clamp(reductionFactor, 0.0, 1.0);

        // Square and reduce the axes
        double modifiedY = square(translationY * clampedReduction);
        double modifiedX = square(translationX * clampedReduction);
        double modifiedRotation = square(rotation * clampedReduction);

        follower.setTeleOpDrive(modifiedX, modifiedY, modifiedRotation, false);
    }

    public void startTeleop() {
        follower.startTeleopDrive();
        follower.setMaxPower(1);
    }

    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public Follower getFollower() {
        return follower;
    }

    public void stopDrivetrain() {
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
    }

    public void resetLocalization() {
        Pose resetPose = new Pose();
        follower.setStartingPose(resetPose);
        follower.setPose(resetPose);
    }

    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        telemetry.addData("X coordinate (meters)", currentPose.getX());
        telemetry.addData("Y coordinate (meters)", currentPose.getY());
        telemetry.addData("Heading angle (radians)", currentPose.getHeading());
    }

    public static double square(double value) {
        return Math.copySign(value * value, value);
    }
}