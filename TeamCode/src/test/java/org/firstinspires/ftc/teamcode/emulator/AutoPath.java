package org.firstinspires.ftc.teamcode.emulator;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;

/** Desktop-emulator counterpart of the production {@code AutoPath}. */
public final class AutoPath {
    private final EmulatorLocalizer localizer;
    private final EmulatorMecanumDrive drivetrain;
    private final Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Keep these values in lockstep with TeamCode/src/main/.../AutoPath.java.
    private final Pose start = poseFactory.of(131.9848, 60.7533, 180);
    private final Pose path2Start = poseFactory.of(131.9848, 60.7533, 180);
    private final Pose path2 = poseFactory.of(23.6697, 32.776, 180);
    private final Pose path2Control1 = poseFactory.of(92.7986, 6.4421, 0);
    private final Pose point2 = poseFactory.of(26.0729, 119.2374, 180);
    private final Pose point3 = poseFactory.of(118.8065, 118.7888, 179.7228);
    private final Pose point4 = poseFactory.of(118.3757, 55.8262, 180);

    private Command activeRoutine;

    AutoPath(EmulatorLocalizer localizer, EmulatorMecanumDrive drivetrain) {
        this.localizer = localizer;
        this.drivetrain = drivetrain;
        // Reuse the tuned controller, velocity, deceleration, and max-speed constants from the
        // real robot. Only the hardware-dependent localizer/drivetrain are replaced.
        this.follower = new Follower(
                localizer,
                drivetrain,
                new Foresight(EmulatorConstants.FORESIGHT_CONFIG)
        );
    }

    public void init() {
        Scheduler.reset();
        activeRoutine = null;
        drivetrain.stop();
        follower.setPose(start);
        follower.update(0.0);
    }

    public void start() {
        activeRoutine = autoRoutine();
        schedule(activeRoutine);
    }

    public void update(double dtSeconds) {
        localizer.setTickSeconds(dtSeconds);
        follower.update(dtSeconds);
        Scheduler.execute();
    }

    public void stop() {
        Scheduler.reset();
        activeRoutine = null;
        follower.stop();
        drivetrain.stop();
    }

    public void resetToStart() {
        follower.setPose(start);
    }

    public boolean isFinished() {
        return activeRoutine != null && !activeRoutine.isScheduled();
    }

    public Pose pose() {
        return follower.pose();
    }

    public List<String> telemetryLines() {
        Pose pose = follower.pose();
        List<String> lines = new ArrayList<>();
        lines.add(String.format("x: %.3f in", pose.x()));
        lines.add(String.format("y: %.3f in", pose.y()));
        lines.add(String.format("heading: %.2f deg", Math.toDegrees(pose.heading())));

        if (follower.currentPath() != null) {
            lines.add(String.format(
                    "Current path distance remaining: %.3f in",
                    follower.distanceToEndpoint()
            ));
            lines.add("Path number: " + follower.pathIndex());
        }
        return lines;
    }

    // Autonomous routine -- deliberately identical to the production sequence.
    public Command autoRoutine() {
        return sequential(
                follow(follower, path2()),
                follow(follower, path2_2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    public Path path2() {
        return curve(path2Start, path2Control1, path2).linear(path2Start, path2);
    }

    public Path path2_2() {
        return line(path2, point2).constant(point2);
    }

    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return line(point3, point4).constant(point4);
    }
}
