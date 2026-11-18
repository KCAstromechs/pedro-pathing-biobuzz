package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "FlyWheelTunerC", group = "tuning")
public class FlyWheelTunerC extends OpMode {
    public DcMotorEx flywheelMotor;


    public static double highVel = 1250; // TPS
    public static double lowVel = 800; // TPS

    public static double curTargetVel = highVel;

    double kP = 0;

    double kF = 0;

    double minVelocity = Double.MAX_VALUE;

    String [] pidConstantsLabels = {"kP", "kF"};
    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};

    int stepIndex = 5; // starting at 0.0001
    int curConstantIndex = 0;

    boolean flywheelActive = false;
    double lastPIDCalculate = 0.0;

    boolean flyWheelDirectPowerMode = false;

    double manualPower = .1;

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

        if (gamepad1.aWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.rightBumperWasPressed()) {
            curConstantIndex = (curConstantIndex + 1) % pidConstantsLabels.length;
        }

        if (gamepad1.leftBumperWasPressed()) {
            minVelocity = Double.MAX_VALUE;
        }

        if (gamepad1.bWasPressed()){
            flyWheelDirectPowerMode = !flyWheelDirectPowerMode;
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

        if (gamepad1.dpadRightWasPressed()) {
            if (manualPower < 1) {
                manualPower += .1;
            }
        }

        if(gamepad1.dpadLeftWasPressed()) {
            if (manualPower > 0) {
                manualPower -= .1;
            }
        }

        // toggle launcher
        if (gamepad1.xWasPressed()) {
            flywheelActive = !flywheelActive;
        }
        if (flywheelActive) {
            lastPIDCalculate = pidCalculate();
            minVelocity = Math.min(flywheelMotor.getVelocity(), minVelocity);
            if (!flyWheelDirectPowerMode) {
            flywheelMotor.setPower(lastPIDCalculate);
            } else {
                flywheelMotor.setPower(manualPower);
            }
        } else {
            lastPIDCalculate = 0;
            flywheelMotor.setPower(0);
        }

        // TELEMETRY
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double curVel = getVelocityTPS();
        double error = curTargetVel - curVel;
        String currentConstant = pidConstantsLabels[curConstantIndex];
        double currentConstantVal = 0;
        if (currentConstant.equals("kP")) {
            currentConstantVal = kP;
        } else if (currentConstant.equals("kF")) {
            currentConstantVal  = kF;
        }
        telemetry.addLine("Velocity currently measured in TicksPerSec");
        telemetry.addData("Target Velocity", curTargetVel);
        telemetry.addData("Minimum Velocity", minVelocity);
        telemetry.addData("Battery Voltage ", voltage);
        telemetry.addData("CurrentVelocity", "%.2f", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Is flywheel active?", flywheelActive);
        telemetry.addData("Last PID calc output", lastPIDCalculate);
        telemetry.addData("Manual Power at ", manualPower);
        telemetry.addLine("------------------------------");
        telemetry.addData("Current Tuning Constant (A Button)", currentConstant);
        telemetry.addData("Current constant value", "%.5f (D-Pad U/D)", currentConstantVal);
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);
        telemetry.addLine("------------------------------");
        telemetry.addData("kP", "%.5f", kP);
        telemetry.addData("kF", "%.5f", kF);
    }

    /**
     * Current returns velocity in tick per second
     * @return
     */
    private double getVelocityTPS() {
        return flywheelMotor.getVelocity();
    }

    /**
     * Returns pid calculation
     * @return pid calculation based on `curTargetVel - getVelocityRPM()`
     */
    private double pidCalculate() {
        double pidCalc = (kF * curTargetVel) + (kP * (curTargetVel - getVelocityTPS()));
        return (Math.max(0, Math.min(1, pidCalc))); //clamp power 0-1
    }

    @Override
    public void stop() {
        flywheelMotor.setPower(0);
    }

}
