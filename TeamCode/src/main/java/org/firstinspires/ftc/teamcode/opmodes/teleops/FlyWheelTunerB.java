package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Config
@TeleOp(name = "FlyWheelTunerB (from Brogan)", group = "tuning")
public class FlyWheelTunerB extends RobotOpMode {
    public DcMotorEx flywheelMotor;

    public static double highVel = 1000; // RPM
    public static double lowVel = 800; // RPM

    public static double curTargetVel = highVel;

    double kP = 0;
    double kF = 0;

    String [] pidConstantsLabels = {"kP", "kF"};
    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};

    int stepIndex = 1; // starting at 1.0 not 10.0
    int curConstantIndex = 0;

    boolean flywheelActive = false;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TODO confirm direction

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

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
            if (curConstantIndex == 0) {
                kP -= stepSizes[stepIndex];
            } else if (curConstantIndex == 1) {
                kF -= stepSizes[stepIndex];
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (curConstantIndex == 0) {
                kP += stepSizes[stepIndex];
            } else if (curConstantIndex == 1) {
                kF += stepSizes[stepIndex];
            }
        }

        // update pid after change occurs
        pidUpdate();

        // toggle/power launcher
        if (gamepad1.xWasPressed()) {
            flywheelActive = !flywheelActive;
        }
        if (flywheelActive) {
            flywheelMotor.setVelocity(curTargetVel);
        } else {
            flywheelMotor.setVelocity(0);
        }

        // TELEMETRY
        double curVel = getVelocity();
        double error = curTargetVel - curVel;
        String currentConstant = pidConstantsLabels[curConstantIndex];
        double currentConstantVal = 0;
        if (currentConstant.equals("kP")) {
            currentConstantVal = kP;
        } else if (currentConstant.equals("kF")) {
            currentConstantVal = kF;
        }

        telemetry.addLine("Velocity in TicksPerSecond");
        telemetry.addData("Target Velocity", curTargetVel);
        telemetry.addData("CurrentVelocity", "%.2f", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Is flywheel active?", flywheelActive);
        telemetry.addLine("------------------------------");
        telemetry.addData("Current Tuning Constant (A Button)", currentConstant);
        telemetry.addData("Current constant value", "%.5f (D-Pad U/D)", currentConstantVal);
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("------------------------------");
        telemetry.addData("kP", "%.5f", kP);
        telemetry.addData("kF", "%.5f", kF);
        telemetry.addData("kP from controller", "%.5f", flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
        telemetry.addData("kF from controller", "%.5f", flywheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
    }

    private double getVelocity() {
        return flywheelMotor.getVelocity();
    }
    private double getVelocityRPM() {
        return flywheelMotor.getVelocity(AngleUnit.DEGREES) / 6; // * 60sec/min / 360(degrees/rot)
    }

    /**
     * Force updates pidfCoefficients of `flywheelMotor`
     */
    private void pidUpdate() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

}
