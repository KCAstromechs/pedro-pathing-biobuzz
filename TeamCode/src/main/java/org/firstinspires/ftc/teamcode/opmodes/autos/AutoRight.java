
package org.firstinspires.ftc.teamcode.opmodes.autos;

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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Disabled
@Autonomous(name = "AutoTemplate", group = "Autonomous")
public class AutoRight extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.75, 8.75, 90);
    private final Pose path1 = poseFactory.of(16.75, 12.75, 265);
    private final Pose path1Control1 = poseFactory.of(56.75, 19.75, 0);
    private final Pose path1Segment1Start = poseFactory.of(16.75, 12.75, 90);
    private final Pose path1Segment1End = poseFactory.of(16.75, 12.75, 180);
    private final Pose path1Segment2Start = poseFactory.of(16.75, 12.75, 180);
    private final Pose path1Segment2End = poseFactory.of(16.75, 12.75, 265);
    private final Pose point2 = poseFactory.of(8.75, 9.75, 270);
    private final Pose point2Segment1Start = poseFactory.of(8.75, 9.75, 265);
    private final Pose point2Segment1End = poseFactory.of(8.75, 9.75, 270);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential( // TODO Add commands here
            follow(follower, toGate()),
            follow(follower, intakeFromGate())
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

    public Path toGate() {
        return Paths.curve(start, path1Control1, path1).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(path1Segment1Start, path1Segment1End)).until(1, Interpolator.linear(path1Segment2Start, path1Segment2End)));
    }

    public Path intakeFromGate() {
        return Paths.line(path1, point2).heading(Interpolator.piecewise().until(1, Interpolator.linear(point2Segment1Start, point2Segment1End)));
    }
}
