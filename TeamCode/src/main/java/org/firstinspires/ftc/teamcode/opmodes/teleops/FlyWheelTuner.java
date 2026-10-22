package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Config
@TeleOp(name = "FlyWheelTuner", group = "tuning")
public class FlyWheelTuner extends RobotOpMode {
    public DcMotorEx flywheelMotor;


    public static double highVel = 1000; // RPM
    public static double lowVel = 800; // RPM

    public static double curTargetVel = highVel;

    double kP = 0;
    double kS = 0;
    double kV = 0;

    String [] pidConstantsLabels = {"kP", "kS", "kV"};
    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};

    int stepIndex = 1; // starting at 1.0 not 10.0
    int curConstantIndex = 0;

    boolean flywheelActive = false;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE); // TODO confirm direction

        telemetry.addLine("Initialization Complete");
    }

    @Override
    public void loop() {
        // get gamepad commands
        // set target vel
        // update telemetry

        if (gamepad1.yWasPressed()) {
            if (curTargetVel == highVel) {
                curTargetVel = lowVel;
            } else { curTargetVel = highVel; }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.aWasPressed()) {
            curConstantIndex = (curConstantIndex + 1) % pidConstantsLabels.length;
        }

        if (gamepad1.dpadDownWasPressed()) {
            switch (curConstantIndex) {
                case (0):
                    kP -= stepSizes[stepIndex];
                case (1):
                    kS -= stepSizes[stepIndex];
                case (2):
                    kV -= stepSizes[stepIndex];
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            switch (curConstantIndex) {
                case (0):
                    kP += stepSizes[stepIndex];
                case (1):
                    kS += stepSizes[stepIndex];
                case (2):
                    kV += stepSizes[stepIndex];
            }
        }

        // toggle launcher
        if (gamepad1.xWasPressed()) {
            flywheelActive = !flywheelActive;
        }
        if (flywheelActive) {
            flywheelMotor.setPower(pidCalculate());
        }

        // TELEMETRY
        double curVel = getVelocityRPM();
        double error = curTargetVel - curVel;
        String currentConstant = pidConstantsLabels[curConstantIndex];
        double currentConstantVal = 0;
        switch (currentConstant) {
            case ("kP"):
                currentConstantVal = kP;
            case ("kS"):
                currentConstantVal = kS;
            case ("kV"):
                currentConstantVal = kV;
        }

        telemetry.addData("Target Velocity", curTargetVel);
        telemetry.addData("CurrentVelocity", "%.2f", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("------------------------------");
        telemetry.addData("Current Tuning Constant", "(A Button)", currentConstant);
        telemetry.addData("Current constant value", "%.5f (D-Pad U/D)", currentConstantVal);
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("------------------------------");
        telemetry.addData("kP", "%.5f", kP);
        telemetry.addData("kS", "%.5f", kS);
        telemetry.addData("kV", "%.5f", kV);
    }

    private double getVelocityRPM() {
        return flywheelMotor.getVelocity(AngleUnit.DEGREES) / 6; // * 60sec/min / 360(degrees/rot)
    }

    private double pidCalculate() {
        return kP * (curTargetVel - getVelocityRPM()) + kV * curTargetVel + kS * Math.signum(curTargetVel);
    }

}
