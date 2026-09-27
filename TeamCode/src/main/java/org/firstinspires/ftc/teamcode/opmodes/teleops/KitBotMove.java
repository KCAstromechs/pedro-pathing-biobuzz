package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@TeleOp(name = "KitBotMove", group = "testing")
@Config
public class KitBotMove extends RobotOpMode {
    @Override
    public void init() {
        super.init();

        robot.drivetrain.usePreviousStartingPose();
        //robot.turret.usePreviousStartingAngle();
    }

    @Override
    public void start() {
        robot.intake.off().schedule();
    }

    @Override
    public void loop() {
        // keybinds

        if (Math.abs(gamepad1.right_stick_x) >= 0.1) robot.drivetrain.unlockHeading();

        robot.drivetrain.arcadeDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                Alliance.current
        );

        // Pose turretPose = ...

        // Tractor Beam.aimTurret()
        if (gamepad1.rightBumperWasPressed()) robot.intake.on().schedule();
        if (gamepad1.rightBumperWasReleased()) robot.intake.off().schedule();
        if (gamepad1.leftBumperWasPressed()) robot.intake.shortReverse().schedule();


        if (gamepad2.leftBumperWasPressed()) Alliance.current = Alliance.RED;
        if (gamepad2.rightBumperWasPressed()) Alliance.current = Alliance.BLUE;

        // Reverse side for field centric button?
        if (gamepad2.xWasPressed()) robot.drivetrain.setPose(
                Alliance.current == Alliance.RED ? new Pose(8.1, 7.5, 0) : new Pose(141.5 - 8.1, 7.5, Math.PI)
        );


        if (gamepad2.bWasPressed()) robot.drivetrain.setPose(robot.drivetrain.getPose().withHeading(
                Alliance.current == Alliance.RED ? 0 : Math.PI
        ));

        robot.telemetry.addData("Alliance", Alliance.current);

        super.loop();
    }
}
