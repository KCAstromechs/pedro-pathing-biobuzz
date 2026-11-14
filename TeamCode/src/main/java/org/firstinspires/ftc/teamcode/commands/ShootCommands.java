package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

public class ShootCommands {

    public static Command startShoot(Robot robot) {
        return sequential(
                instant(robot.flywheel::turnOn),
                loop(
                    sequential(
                        waitUntil(robot.flywheel::atTarget),
                        instant(robot.kicker::turnOn),
                        waitUntil(() -> !robot.flywheel.atTarget()),
                        instant(robot.kicker::turnOff)
                    )
                ).until(robot.flywheel::isOff) // I think this is interruptable by scheduling the stopShoot command
        );                                     // or by putting this command in a race composition
    }

    public static Command stopShoot(Robot robot) {
        return sequential(
                instant(robot.flywheel::turnOff),
                instant(robot.kicker::turnOff)
        );
    }

    public static Command reverseShoot(Robot robot) {
        return sequential(
                instant(robot.flywheel::turnReverse),
                instant(robot.kicker::turnReverse)
        );
    }

}
