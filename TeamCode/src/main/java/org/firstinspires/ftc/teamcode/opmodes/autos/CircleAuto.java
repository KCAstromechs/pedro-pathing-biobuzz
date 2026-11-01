package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

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

@Autonomous(name = "CircleAuto", group = "Autonomous")
public class CircleAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(70.75, 117.92, 0);
    private final Pose point1 = poseFactory.of(118.0437, 70.5873, -89.6511);
    private final Pose point1Control1 = poseFactory.of(117.996, 117.3915, 0);
    private final Pose point2 = poseFactory.of(70.75, 23.58, 0.9919);
    private final Pose point2Control1 = poseFactory.of(118.3373, 24.1706, 0);
    private final Pose point3 = poseFactory.of(23.58, 70.75, -90.1441);
    private final Pose point3Control1 = poseFactory.of(23.2235, 23.9669, 0);
    private final Pose point4 = poseFactory.of(70.75, 118, -179.4544);
    private final Pose point4Control1 = poseFactory.of(24.1217, 117.7923, 0);
    private final Pose group1Start = poseFactory.of(70.75, 117.92, 0);
    private final Pose group1End = poseFactory.of(70.75, 118, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
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

    public Path path1() {
        return Paths.path(Paths.curve(start, point1Control1, point1), Paths.curve(point1, point2Control1, point2), Paths.curve(point2, point3Control1, point3), Paths.curve(point3, point4Control1, point4)).tangent();
    }
}