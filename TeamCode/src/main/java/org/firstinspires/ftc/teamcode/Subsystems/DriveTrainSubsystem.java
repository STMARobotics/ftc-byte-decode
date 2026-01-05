package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.core.math.MathUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Subsystem that controls the robot's drivetrain using Pedro Pathing.
 * Handles both teleop driving and path following capabilities.
 */
public class DriveTrainSubsystem extends SubsystemBase {

    private final Follower follower;
    private final Telemetry telemetry;
    private Pose currentPose = new Pose();

    /**
     * Constructs a new DriveTrainSubsystem.
     *
     * @param hardwareMap the hardware map from the OpMode
     * @param telemetry   the telemetry instance for logging data
     */
    public DriveTrainSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        follower = Constants.createFollower(hardwareMap);
    }

    /**
     * Drives the robot using field-centric or robot-centric control.
     *
     * @param translationX    the x-axis (strafe) input, typically from -1.0 to 1.0
     * @param translationY    the y-axis (forward/backward) input, typically from -1.0 to 1.0
     * @param rotation        the rotation input, typically from -1.0 to 1.0
     * @param reductionFactor speed reduction factor clamped between 0.0 and 1.0
     */
    public void drive(double translationX, double translationY, double rotation, double reductionFactor) {
        double clampedReduction = MathUtils.clamp(reductionFactor, 0.0, 1.0);

        // Square and reduce the axes
        double modifiedY = square(translationY * clampedReduction);
        double modifiedX = square(translationX * clampedReduction);
        double modifiedRotation = square(rotation * clampedReduction * .7);

        follower.setTeleOpDrive(modifiedX, modifiedY, modifiedRotation, false);
    }

    /**
     * Initializes the drivetrain for teleop control mode.
     * Sets maximum power to full speed.
     */
    public void startTeleop() {
        follower.startTeleopDrive();
        follower.setMaxPower(1);
    }

    /**
     * Creates a new PathBuilder for constructing autonomous paths.
     *
     * @return a PathBuilder instance from the follower
     */
    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    /**
     * Gets the underlying Pedro Pathing Follower instance.
     *
     * @return the Follower used by this subsystem
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Stops all drivetrain motors immediately.
     */
    public void stopDrivetrain() {
        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
    }

    /**
     * Resets the robot's localization to the origin position (0, 0, 0).
     */
    public void resetLocalization() {
        Pose resetPose = new Pose();
        follower.setStartingPose(resetPose);
        follower.setPose(resetPose);
    }

    /**
     * Periodic method called every loop iteration.
     * Updates the follower and logs current pose to telemetry.
     */
    @Override
    public void periodic() {
        follower.update();
        currentPose = follower.getPose();
        telemetry.addData("X coordinate (inches)", currentPose.getX());
        telemetry.addData("Y coordinate (inches)", currentPose.getY());
        telemetry.addData("Heading angle (radians)", currentPose.getHeading());
    }

    /**
     * Squares a value while preserving its sign.
     * Used for input curve adjustment to provide finer control at low speeds.
     *
     * @param value the input value to square
     * @return the squared value with the original sign
     */
    public static double square(double value) {
        return Math.copySign(value * value, value);
    }
}