
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

import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Autonomous(name = "AutoLeft", group = "Autonomous")
public class AutoLeft extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.6411, 134.0226, 90);
    private final Pose toPark = poseFactory.of(12.75, 118.75, 90);
    private final Pose toParkControl1 = poseFactory.of(58.75, 118.75, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential( // TODO Add commands here
            waitMs(1700),
            robot.intake.on(),
            ShootCommands.startShoot(robot).raceWith(waitMs(8000)),
            ShootCommands.stopShoot(robot),
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

    public Path toPark() {
        return Paths.curve(start, toParkControl1, toPark).linear(start, toPark)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(1.0)
                );
    }
}
