package org.firstinspires.ftc.teamcode.emulator;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import java.util.Collections;
import java.util.Map;

import emulator.sim.MecanumRobot;

/**
 * Uses the emulator's physics pose as Pedro's localizer.
 *
 * <p>Pedro uses the conventional 0..144 inch field coordinates while emulator v1.0.3 uses a
 * field-centered -72..72 inch coordinate system. Headings use radians in both systems.</p>
 */
final class EmulatorLocalizer implements Localizer {
    private static final double FIELD_CENTER_INCHES = 72.0;

    private final MecanumRobot robot;
    private MotionState motionState = MotionState.zero();
    private double tickSeconds = 0.05;

    EmulatorLocalizer(MecanumRobot robot) {
        this.robot = robot;
    }

    void setTickSeconds(double tickSeconds) {
        if (tickSeconds > 0.0) {
            this.tickSeconds = tickSeconds;
        }
    }

    @Override
    public void setPose(Pose pose) {
        robot.resetPose(toEmulatorPose(pose));
        motionState = MotionState.ofVelocity(pose, Velocity.zero());
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    @Override
    public void update() {
        Pose previous = motionState.pose();
        emulator.sim.Pose simulated = robot.getPose();

        double headingDelta = wrapAngle(simulated.getHeadingRad() - previous.heading());
        Pose current = new Pose(
                simulated.getX() + FIELD_CENTER_INCHES,
                simulated.getY() + FIELD_CENTER_INCHES,
                previous.heading() + headingDelta
        );

        Velocity velocity = new Velocity(
                (current.x() - previous.x()) / tickSeconds,
                (current.y() - previous.y()) / tickSeconds,
                headingDelta / tickSeconds
        );
        motionState = MotionState.ofVelocity(current, velocity);
    }

    @Override
    public void reset() {
        setPose(Pose.zero());
    }

    @Override
    public Map<String, Object> debug() {
        return Collections.<String, Object>singletonMap("source", "ftc-control-hub-emulator");
    }

    static emulator.sim.Pose toEmulatorPose(Pose pose) {
        return new emulator.sim.Pose(
                pose.x() - FIELD_CENTER_INCHES,
                pose.y() - FIELD_CENTER_INCHES,
                pose.heading()
        );
    }

    private static double wrapAngle(double radians) {
        double wrapped = radians % (2.0 * Math.PI);
        if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        } else if (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }
}
