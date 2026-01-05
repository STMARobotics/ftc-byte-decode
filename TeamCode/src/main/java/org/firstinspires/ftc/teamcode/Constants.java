package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.Math.LookupTableMath;

/**
 * Contains all robot constants organized by subsystem.
 * This class cannot be instantiated.
 */
public final class Constants {

    /**
     * Constants for the drivetrain motors.
     */
    public static final class DriveTrainConstants {
        /** Hardware name for the front right motor */
        public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";

        /** Hardware name for the front left motor */
        public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";

        /** Hardware name for the back right motor (port 2) */
        public static final String BACK_RIGHT_MOTOR_NAME = "backRight";

        /** Hardware name for the back left motor */
        public static final String BACK_LEFT_MOTOR_NAME = "backLeft";
    }

    /**
     * Constants for sensors used on the robot.
     */
    public static final class SensorConstants {
        /** Hardware name for the SparkFun OTOS odometry sensor */
        public static final String SPARKFUN_OTOS_NAME = "Odometry Device";

        /** Offset position of the OTOS sensor from robot center (x, y in inches, heading in radians) */
        public static final SparkFunOTOS.Pose2D OTOS_OFFSET = new SparkFunOTOS.Pose2D(0.875, 0, -Math.PI/2);
    }

    /**
     * Constants for the turret mechanism.
     */
    public static final class TurretConstants {
        /** Time in seconds for a shooting action */
        public static final double SHOOTING_TIME = 0.5;

        /** Maximum turret rotation in degrees */
        public static final double TURRET_MAX_DEGREE = 145.0;

        /** Minimum turret rotation in degrees */
        public static final double TURRET_MIN_DEGREE = -145.0;

        /** Minimum voltage reading from the potentiometer */
        public static final double POTENTIOMETER_MIN_VOLTAGE = 0.9;

        /** Maximum voltage reading from the potentiometer */
        public static final double POTENTIOMETER_MAX_VOLTAGE = 1.8;

        /** Proportional gain for turret position control */
        public static final double TURRET_KP = 0.02;

        /** Derivative gain for turret position control */
        public static final double TURRET_KD = 0.0;

        /** Tolerance in degrees for turret position targeting */
        public static final double TURRET_DEGREE_TOLERANCE = 2.5;
    }

    /**
     * Constants for the intake mechanism.
     */
    public static final class IntakeConstants {
        /** Hardware name for the intake motor */
        public static final String INTAKE_MOTOR_NAME = "intakeMotor";

        /** Hardware name for the intake distance sensor */
        public static final String INTAKE_SENSOR_NAME = "intakeDistanceSensor";

        /** Intake motor speed as a percentage (0 to 1) */
        public static final double INTAKE_SPEED = 1;

        /** Distance threshold in inches for detecting a game element */
        public static final double INTAKE_SENSOR_DISTANCE = 6.0;
    }

    /**
     * Constants for the indexer mechanism.
     */
    public static final class IndexerConstants {
        /** Hardware name for the wheel indexer motor */
        public static final String WHEEL_INDEXER_MOTOR_NAME = "wheelIndexerMotor";

        /** Hardware name for the belt indexer motor */
        public static final String BELT_INDEXER_MOTOR_NAME = "beltIndexerMotor";

        /** Hardware name for the wheel distance sensor */
        public static final String WHEEL_SENSOR_NAME = "wheelDistanceSensor";

        /** Hardware name for the belt distance sensor */
        public static final String BELT_SENSOR_NAME = "beltDistanceSensor";

        /** Distance threshold in inches for the wheel sensor */
        public static final double WHEEL_SENSOR_DISTANCE = 3.2;

        /** Distance threshold in inches for the belt sensor */
        public static final double BELT_SENSOR_DISTANCE = 7.0;

        /** Belt indexer motor speed as a percentage (0 to 1) */
        public static final double BELT_INDEXING_SPEED = 1.0;

        /** Wheel indexer motor speed as a percentage (0 to 1) */
        public static final double WHEEL_INDEXING_SPEED = 1.0;
    }

    /**
     * Lookup table for interpolating shooter speed based on Limelight tx values.
     * Maps tx (offset) to shooter velocity in ticks per second.
     */
    public static final LookupTableMath INTERPOLATOR = new LookupTableMath()
            .addEntry(14.9, 1100)
            .addEntry(4.9, 1100)
            .addEntry(-1.2, 1100)
            .addEntry(-5.0, 1200)
            .addEntry(-8.54, 1200)
            .addEntry(-9.6, 1250)
            .addEntry(-10.6, 1352)
            .addEntry(-13.5, 1490);
}
