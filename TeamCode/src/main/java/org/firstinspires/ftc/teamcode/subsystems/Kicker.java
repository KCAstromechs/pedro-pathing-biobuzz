package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.*;

@Config
public class Kicker {
    private boolean slowMode = false;
    private Mode mode = Mode.OFF;
    public static double fastPower = -1;
    public static double slowPower = -1;
    public static double offPower = 0;
    public static double reversePower = 1;
    public static double shortReverseTimeMs = 150;

    private final CRServo kickerServo;

    private final Telemetry telemetry;

    public Kicker(Robot robot) {
        kickerServo = robot.hardwareMap.get(CRServo.class, "kicker");
        kickerServo.setDirection(CRServo.Direction.FORWARD);
        telemetry = robot.telemetry;
    }

    public void turnOn() {
        mode = Mode.ON;
    }

    public void turnOff() {
        mode = Mode.OFF;
    }

    public void reverse() {
        mode = Mode.REVERSE;
    }

    public Command shortReverse() {
        return instant(() -> reverse())
                .then(waitMs(shortReverseTimeMs))
                .then(instant(() -> turnOn()));
    }

    public void toggle() {
        if (mode == Mode.OFF) {
            turnOn();
        } else {
            turnOff();
        }
    }

    public void slowDown() {
        slowMode = true;
    }

    public void speedUp() {
        slowMode = false;
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case ON:
                    kickerServo.setPower(slowMode ? slowPower : fastPower);
                    break;
                case OFF:
                    kickerServo.setPower(offPower);
                    break;
                case REVERSE:
                    kickerServo.setPower(reversePower);
                    break;
            }
        });
    }

    enum Mode {
        ON,
        OFF,
        REVERSE
    }
}