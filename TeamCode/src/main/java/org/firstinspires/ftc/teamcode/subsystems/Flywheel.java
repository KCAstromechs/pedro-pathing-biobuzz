package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.math.TurretLocation;
//import org.firstinspires.ftc.teamcode.math.WaveLength;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public class Flywheel {
    public static double kP = 0.01;
    public static double kS = 0.065;
    public static double kV = 0.000365;
    public static int velocityTolerance = 25;
    public static boolean override = false;
    public static double overrideTarget = 1000;
    private final DcMotorEx flywheelMotorTop;
    private final DcMotorEx flywheelMotorBottom;
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private boolean tempOverride = false;
    private boolean on = false;
    private double target = 0;

    public Flywheel(Robot robot) {
        flywheelMotorTop = robot.hardwareMap.get(DcMotorEx.class, "flywheelTop");
        flywheelMotorBottom = robot.hardwareMap.get(DcMotorEx.class, "flywheelBottom");
        flywheelMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = robot.telemetry;
        drivetrain = robot.drivetrain;
    }

    public void setTarget(double target) {
        this.target = target;
        tempOverride = true;
    }

    public void useInterpolation() {
        tempOverride = false;
    }

    private void setPower(double power) {
        flywheelMotorTop.setPower(power);
        flywheelMotorBottom.setPower(power);
    }

    private double getVelocity() {
        return flywheelMotorBottom.getVelocity();
    }

    public boolean atTarget() {
        return Math.abs(target - getVelocity()) <= velocityTolerance;
    }

    public void turnOn() {
        on = true;
    }

    public void turnOff() {
        on = false;
    }

    public void toggle() {
        on = !on;
        if (on) turnOn();
        else turnOff();
    }

    public Command periodic() {
        return infinite(() -> {
            if (on) {
//                target = override ? overrideTarget : (tempOverride ? target : WaveLength.getVelocityWithInterpolation(TurretLocation.getTurretPose(drivetrain.getPose()), Alliance.current));
                target = 10;
                setPower(kP * (target - getVelocity()) + kV * target + kS * Math.signum(target));
            } else {
                target = 0;
                setPower(0);
            }

            telemetry.addData("Flywheel Velocity", getVelocity());
            telemetry.addData("Flywheel Target", target);
            telemetry.addData("Flywheel Power", flywheelMotorBottom.getPower());
        });
    }
}