
package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Autonomous(name = "BlueLeft", group = "Autonomous")
public class BlueLeft extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(132, 118.5, 180);
    private final Pose path1Start = poseFactory.of(132, 118.5, 180);
    private final Pose path1 = poseFactory.of(84, 118.5, 270);
    private final Pose point2 = poseFactory.of(127, 94, 0);
    private final Pose point3 = poseFactory.of(123, 94, 0);
    private final Pose point4 = poseFactory.of(123, 133, 0);
    private final Pose point5 = poseFactory.of(129, 133, 0);
    private final Pose point6 = poseFactory.of(129, 128, 0);
    private final Pose point7 = poseFactory.of(84, 118.5, 270);
    private final Pose point7Control1 = poseFactory.of(84, 128, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, toHive()),
                waitMs(1000),
                follow(follower, toFlower()),
                robot.intake.on(),
                waitMs(1000),
                robot.intake.off(),
                follow(follower, backUp()),
                follow(follower, toGarden()),
                robot.intake.on(),
                follow(follower, collectGarden()),
                robot.intake.off(),
                follow(follower, backUp2()),
                follow(follower, toHive2())
        );
    }

    @Override
    public void init() {
        super.init();

        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        Alliance.current = Alliance.BLUE;
    }

    @Override
    public void start() {
        autoRoutine().schedule();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());

        if (follower.currentPath() != null) {
            telemetry.addData(
                    "Current path distance remaining",
                    follower.distanceToEndpoint()
            );
            telemetry.addData("Path number", follower.pathIndex());
        }

        telemetry.update();
    }

    public Path toHive() {
        return Paths.line(path1Start, path1).linear(path1Start, path1);
    }

    public Path toFlower() {
        return Paths.line(path1, point2).linear(path1, point2);
    }

    public Path backUp() {
        return Paths.line(point2, point3).linear(point2, point3);
    }

    public Path toGarden() {
        return Paths.line(point3, point4).linear(point3, point4);
    }

    public Path collectGarden() {
        return Paths.line(point4, point5).tangent();
    }

    public Path backUp2() {
        return Paths.line(point5, point6).linear(point5, point6);
    }

    public Path toHive2() {
        return Paths.curve(point6, point7Control1, point7)
                .linear(point6, point7);
    }
}
