
package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.ivy.commands.Commands.waitMs;
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

@Autonomous(name = "AutoLeftJustPark", group = "Autonomous")
public class AutoLeftJustPark extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(10.75, 133.75, 90);
    private final Pose toPark = poseFactory.of(10.75, 117.75, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
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
        return Paths.line(start, toPark).linear(start, toPark);
    }
}
