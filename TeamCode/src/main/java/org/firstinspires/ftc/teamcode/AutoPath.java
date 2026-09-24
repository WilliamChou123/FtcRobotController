package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(131.9848, 60.7533, 180);
    private final Pose path2Start = poseFactory.of(131.9848, 60.7533, 180);
    private final Pose path2 = poseFactory.of(23.6697, 32.776, 180);
    private final Pose path2Control1 = poseFactory.of(92.7986, 6.4421, 0);
    private final Pose point2 = poseFactory.of(26.0729, 119.2374, 180);
    private final Pose point3 = poseFactory.of(118.8065, 118.7888, 179.7228);
    private final Pose point4 = poseFactory.of(118.3757, 55.8262, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path2()),
                follow(follower, path2_2()),
                follow(follower, path3()),
                follow(follower, path4())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
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