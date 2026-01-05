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

/**
 * Configuration constants for the Pedro Pathing library.
 * <p>
 * This class provides all the necessary configuration for path following,
 * including drivetrain settings, localization parameters, and motion constraints.
 * It serves as the central configuration point for the Pedro Pathing system.
 * </p>
 */
public class Constants {

    /**
     * Default path constraints defining maximum velocity, acceleration,
     * angular velocity, and angular acceleration for path following.
     */
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Creates and configures a new Follower instance for path following.
     * <p>
     * Builds a follower with the configured path constraints, mecanum drivetrain
     * settings, and OTOS localizer configuration.
     * </p>
     *
     * @param hardwareMap the hardware map from the OpMode for accessing hardware devices
     * @return a fully configured {@link Follower} instance ready for path following
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }

    /**
     * Follower configuration constants including robot mass, deceleration
     * characteristics, and PID coefficients for translational and heading control.
     */
    public static final FollowerConstants followerConstants = new FollowerConstants()
            .mass(15.8)
            .forwardZeroPowerAcceleration(-58.742)
            .lateralZeroPowerAcceleration(-82.934)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.0, 0));

    /**
     * Mecanum drivetrain configuration including motor names, directions,
     * maximum power, and velocity characteristics in X and Y directions.
     */
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(DriveTrainConstants.FRONT_RIGHT_MOTOR_NAME)
            .rightRearMotorName(DriveTrainConstants.BACK_RIGHT_MOTOR_NAME)
            .leftRearMotorName(DriveTrainConstants.BACK_LEFT_MOTOR_NAME)
            .leftFrontMotorName(DriveTrainConstants.FRONT_LEFT_MOTOR_NAME)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65)
            .yVelocity(52);

    /**
     * SparkFun OTOS (Optical Tracking Odometry Sensor) localizer configuration
     * including hardware name, measurement units, and sensor offset from robot center.
     */
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName(SensorConstants.SPARKFUN_OTOS_NAME)
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(SensorConstants.OTOS_OFFSET);

}