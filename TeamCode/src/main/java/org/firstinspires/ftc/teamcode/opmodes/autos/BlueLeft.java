package org.firstinspires.ftc.teamcode.opmodes.autos;

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

import java.lang.invoke.ConstantCallSite;

@Autonomous(name = "BlueLeft", group = "Autonomous")
public class BlueLeft extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(132, 118.5, 180);
    private final Pose path1Start = poseFactory.of(132, 118.5, 180);
    private final Pose path1 = poseFactory.of(84, 118.5, 270);
    private final Pose point2 = poseFactory.of(129, 94, 0);
    private final Pose point3 = poseFactory.of(123, 94, 0);
    private final Pose point4 = poseFactory.of(123, 133, 0);
    private final Pose point5 = poseFactory.of(131, 133, 0);
    private final Pose point6 = poseFactory.of(131, 128, 0);
    private final Pose point7 = poseFactory.of(84, 118.5, 270);
    private final Pose point7Control1 = poseFactory.of(84, 128, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
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
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return line(path1, point2).linear(path1, point2);
    }

    public Path path3() {
        return line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return line(point3, point4).linear(point3, point4);
    }

    public Path path5() {
        return line(point4, point5).tangent();
    }

    public Path path6() {
        return line(point5, point6).linear(point5, point6);
    }

    public Path path7() {
        return curve(point6, point7Control1, point7).linear(point6, point7);
    }
}
