
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Disabled
@Autonomous(name = "AutoTemplate", group = "Autonomous")
public class AutoTemplate extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose start; // TODO Add points here

    // Autonomous routine
    public Command autoRoutine() {
        return sequential( // TODO Add commands here
//                follow(follower, toHive()),
//                waitMs(1000),
//                follow(follower, toFlower()),
//                robot.intake.on(),
//                waitMs(1000),
//                robot.intake.off(),
//                follow(follower, backUp()),
//                follow(follower, toGarden()),
//                robot.intake.on(),
//                follow(follower, collectGarden()),
//                robot.intake.off(),
//                follow(follower, backUp2()),
//                follow(follower, toHive2())
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

    // TODO write path commands here
}
