package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Blocker {
    private Mode mode = Mode.BLOCK;
    public static double blockPosition = 0.1;
    public static double unblockPosition = 0.35;
    private final Servo blockerServo;
    private final Telemetry telemetry;

    public Blocker(Robot robot) {
        blockerServo = robot.hardwareMap.get(Servo.class, "blocker");
        telemetry = robot.telemetry;
    }

    public void setBlocking() {
        mode = Mode.BLOCK;
    }

    public void setUnblocking() {
        mode = Mode.UNBLOCK;
    }

    public Command block() {
        return instant(this::setBlocking).requiring(blockerServo);
    }

    public Command unblock() {
        return instant(this::setUnblocking).requiring(blockerServo);
    }

    public void toggle() {
        if (mode == Mode.BLOCK) {
            setUnblocking();
        } else {
            setBlocking();
        }
    }

    public boolean isBlocking() {
        return (mode == Mode.BLOCK);
    }

    public boolean isUnblocking() {
        return (mode == Mode.UNBLOCK);
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case BLOCK:
                    blockerServo.setPosition(blockPosition);
                    break;
                case UNBLOCK:
                    blockerServo.setPosition(unblockPosition);
                    break;
            }

            telemetry.addLine("-----Blocker Telemetry:");
            telemetry.addData("Is blocking", isBlocking());
        });
    }

    enum Mode {
        BLOCK,
        UNBLOCK
    }
}