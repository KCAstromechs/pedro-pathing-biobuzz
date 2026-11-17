package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Config
@TeleOp(name = "BlockerServoTest", group = "tuning")
public class BlockerServoTest extends OpMode {
    public Servo blockerServo;

    public static double targetPos = 0.0;

    double [] stepSizes = {1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};

    int stepIndex = 1; // starting at .1 not 1.0

    @Override
    public void init() {
        blockerServo = hardwareMap.get(Servo.class, "blocker");

        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        // get gamepad commands
        // set target vel
        // update telemetry

        if (gamepad1.yWasPressed()) {
            blockerServo.setPosition(targetPos);
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadDownWasPressed()) {
            targetPos -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetPos += stepSizes[stepIndex];
        }

        // TELEMETRY
        telemetry.addLine("------------------------------");
        telemetry.addLine("Press (Y) to set position to target position");
        telemetry.addData("Current targetPos", "%.5f (D-Pad U/D)", targetPos);
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("------------------------------");
    }
}
