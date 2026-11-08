
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
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.ShootCommands;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Autonomous(name = "AutoRight", group = "Autonomous")
public class AutoRight extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.75, 8.75, 270);
    private final Pose toGarden = poseFactory.of(8, 12.734, 270);
    private final Pose toGardenControl1 = poseFactory.of(56.438, 16.0442, 0);
    private final Pose eatGardenStart = poseFactory.of(8, 12.734, 270);
    private final Pose eatGarden = poseFactory.of(8, 8.6815, 270);
    private final Pose point3Start = poseFactory.of(8, 8.6815, 270);
    private final Pose point3 = poseFactory.of(56.6225, 15.5785, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential( // TODO Add commands here
            robot.intake.on(),
            ShootCommands.startShoot(robot),
            waitMs(5000),
            ShootCommands.stopShoot(robot),
            instant(() -> robot.kicker.turnOff()),
            instant(() -> robot.flywheel.turnOff()),
            follow(follower, toGarden()),
            follow(follower, eatGarden()),
            waitMs(2000),
            follow(follower, path3()),
            ShootCommands.startShoot(robot),
            waitMs(5000),
            ShootCommands.stopShoot(robot)
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

    public Path toGarden() {
        return Paths.curve(start, toGardenControl1, toGarden).linear(start, toGarden)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.8)
                );
    }

    public Path eatGarden() {
        return Paths.line(eatGardenStart, eatGarden).linear(eatGardenStart, eatGarden)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.3)
                );
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.8)
                );
    }
}
