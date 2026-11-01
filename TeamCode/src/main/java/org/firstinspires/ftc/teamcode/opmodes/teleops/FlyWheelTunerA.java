package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.RobotOpMode;

@Config
@TeleOp(name = "FlyWheelTunerA", group = "tuning")
public class FlyWheelTunerA extends RobotOpMode {
    public DcMotorEx flywheelMotor;


    public static double highVel = 1000; // RPM
    public static double lowVel = 800; // RPM

    public static double curTargetVel = highVel;

    double kP = 0;
    double kS = 0.11;
    double kV = 0.01;

    String [] pidConstantsLabels = {"kP", "kS", "kV"};
    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};

    int stepIndex = 1; // starting at 1.0 not 10.0
    int curConstantIndex = 0;

    boolean flywheelActive = false;
    double lastPIDCalculate = 0.0;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TODO confirm direction

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
                kS -= stepSizes[stepIndex];
            } else if (curConstantIndex == 2) {
                kV -= stepSizes[stepIndex];
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            if (curConstantIndex == 0) {
                kP += stepSizes[stepIndex];
            } else if (curConstantIndex == 1) {
                kS += stepSizes[stepIndex];
            } else if (curConstantIndex == 2) {
                kV += stepSizes[stepIndex];
            }
        }

        // toggle launcher
        if (gamepad1.xWasPressed()) {
            flywheelActive = !flywheelActive;
        }
        if (flywheelActive) {
            lastPIDCalculate = pidCalculate();
            flywheelMotor.setPower(lastPIDCalculate);
        } else {
            flywheelMotor.setPower(0);
        }

        double force_power = gamepad1.right_trigger;
        flywheelMotor.setPower(force_power);

        // TELEMETRY
        double curVel = getVelocityRPM();
        double error = curTargetVel - curVel;
        String currentConstant = pidConstantsLabels[curConstantIndex];
        double currentConstantVal = 0;
        if (currentConstant.equals("kP")) {
            currentConstantVal = kP;
        } else if (currentConstant.equals("kS")) {
            currentConstantVal = kS;
        } else if (currentConstant.equals("kV")) {
            currentConstantVal = kV;
        }

        telemetry.addLine("Velocity currently measured in TicksPerSec");
        telemetry.addData("Target Velocity", curTargetVel);
        telemetry.addData("CurrentVelocity", "%.2f", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Is flywheel active?", flywheelActive);
        telemetry.addData("Last PID calc output", lastPIDCalculate);
        telemetry.addData("Right Trigger val (force power)", force_power);
        telemetry.addLine("------------------------------");
        telemetry.addData("Current Tuning Constant (A Button)", currentConstant);
        telemetry.addData("Current constant value", "%.5f (D-Pad U/D)", currentConstantVal);
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("------------------------------");
        telemetry.addData("kP", "%.5f", kP);
        telemetry.addData("kS", "%.5f", kS);
        telemetry.addData("kV", "%.5f", kV);
    }

    /**
     * Current returns velocity in tick per second
     * @return
     */
    private double getVelocityRPM() {
        return flywheelMotor.getVelocity(); // * 60sec/min / 360(degrees/rot)
    }

    /**
     * Returns pid calculation
     * @return pid calculation based on `curTargetVel - getVelocityRPM()`
     */
    private double pidCalculate() {
        return kP * (curTargetVel - getVelocityRPM()) + kV * curTargetVel + kS * Math.signum(curTargetVel);
    }

    /**
     * Returns pid calculation
     * @param error difference between target velocity and current velocity
     * @return pid calculation based on `error`
     */
    private double pidCalculate(double error) {
        return kP * (error) + kV * curTargetVel + kS * Math.signum(curTargetVel);
    }

}
