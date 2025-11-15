package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public final class Constants {
    public static final class DriveTrainConstants {
        public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";
        public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";
        public static final String BACK_RIGHT_MOTOR_NAME = "backRight";
        public static final String BACK_LEFT_MOTOR_NAME = "backLeft";
    }

    public static final class SensorConstants {
        public static final String SPARKFUN_OTOS_NAME = "Odometry Device";
        public static final SparkFunOTOS.Pose2D OTOS_OFFSET = new SparkFunOTOS.Pose2D(0.09, -0.16, Math.PI/2);
    }

    public static final class TurretConstants {
        public static final int SHOOTING_SPEED = 1300;
        public static final double SHOOTING_TIME = 0.2;
        public static final double TURRET_MAX_DEGREE = 145.0;
        public static final double TURRET_MIN_DEGREE = -145.0;
        public static final double POTENTIOMETER_MIN_VOLTAGE = 0.9;
        public static final double POTENTIOMETER_MAX_VOLTAGE = 1.8;
        public static final double TURRET_KP = 0.028;
        public static final double TURRET_KD = 0.004;
        public static final double TURRET_DEGREE_TOLERANCE = 2;
    }

    public static final class IntakeConstants {
        public static final String INTAKE_MOTOR_NAME = "intakeMotor";
        public static final String INTAKE_SENSOR_NAME = "intakeDistanceSensor";
        public static final double INTAKE_SPEED = 1; // percent of power applied from 0 < speed < 1
        public static final double INTAKE_SENSOR_DISTANCE = 1.0; // inches
    }

    public static final class IndexerConstants {
        public static final String WHEEL_INDEXER_MOTOR_NAME = "wheelIndexerMotor";
        public static final String BELT_INDEXER_MOTOR_NAME = "beltIndexerMotor";
        public static final String WHEEL_SENSOR_NAME = "wheelDistanceSensor";
        public static final String BELT_SENSOR_NAME = "beltDistanceSensor";
        public static final double WHEEL_SENSOR_DISTANCE = 1.0; // inches
        public static final double BELT_SENSOR_DISTANCE = 1.0; // inches
        public static final double BELT_INDEXING_SPEED = 1.0; // percent of power applied from 0 < speed < 1
        public static final double WHEEL_INDEXING_SPEED = 1.0; // percent of power applied from 0 < speed < 1
    }
}
