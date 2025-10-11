package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.Constants.SensorConstants;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }

    public static final FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
            .mass(5)
            .forwardZeroPowerAcceleration(-58.742)
            .lateralZeroPowerAcceleration(-82.934)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.0, 0));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(DriveTrainConstants.FRONT_RIGHT_MOTOR_NAME)
            .rightRearMotorName(DriveTrainConstants.BACK_RIGHT_MOTOR_NAME)
            .leftRearMotorName(DriveTrainConstants.BACK_LEFT_MOTOR_NAME)
            .leftFrontMotorName(DriveTrainConstants.FRONT_LEFT_MOTOR_NAME)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName(SensorConstants.SPARKFUN_OTOS_NAME)
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(SensorConstants.OTOS_OFFSET);
}