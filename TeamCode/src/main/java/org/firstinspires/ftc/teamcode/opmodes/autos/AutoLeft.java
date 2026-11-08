
package org.firstinspires.ftc.teamcode.opmodes.autos;

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

@Autonomous(name = "AutoTemplate", group = "Autonomous")
public class AutoLeft extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.6411, 134.0226, 90);
    private final Pose toFlower = poseFactory.of(51.9022, 131.5723, 95);
    private final Pose toFlowerControl1 = poseFactory.of(57.2828, 121.6769, 0);
    private final Pose toParkStart = poseFactory.of(51.9022, 131.5723, 95);
    private final Pose toPark = poseFactory.of(9.2987, 120.385, 90);
    private final Pose toParkControl1 = poseFactory.of(52.1732, 121.1908, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential( // TODO Add commands here
            follow(follower, toFlower()),
            follow(follower, toPark())
        );
    }

    @Override
    public void init() {
        super.init();

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

    public Path toFlower() {
        return Paths.curve(start, toFlowerControl1, toFlower).linear(start, toFlower);
    }

    public Path toPark() {
        return Paths.curve(toParkStart, toParkControl1, toPark).linear(toParkStart, toPark);
    }
}
