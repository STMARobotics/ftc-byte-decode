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
        public static final int OTOS_LINEAR_SCALAR = 1;
        public static final int OTOS_ANGULAR_SCALAR = 1;
    }

    public static final class ShootingConstants {
        public static final String SHOOTER_MOTOR_NAME = "shooterMotor";
        public static final String INDEXER_MOTOR_NAME = "indexerMotor";
        public static final int SHOOTING_SPEED = 1300;
        public static final double INDEXING_SPEED = 1; // percent of power applied from 0 < speed < 1
    }

    public static final class IntakeConstants {
        public static final String INTAKE_MOTOR_NAME = "intakeMotor";
        public static final double INTAKE_SPEED = 1; // percent of power applied from 0 < speed < 1
    }
}
