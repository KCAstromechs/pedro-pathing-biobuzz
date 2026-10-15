package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.math.TurretLocation;
//import org.firstinspires.ftc.teamcode.math.WaveLength;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public class Flywheel {

    public static class FlywheelParams {
        public static double kP = 0.01;
        public static double kS = 0.065;
        public static double kV = 0.000365;
        public static int velocityTolerance = 25;
        private double shootingVelocity = 1000; //in encoder ticks/sec
        private double targetVelocity = 0;
    }

    public static FlywheelParams params = new FlywheelParams();
    
    public static boolean override = false;
    private final DcMotorEx flywheelMotorTop;
//    private final DcMotorEx flywheelMotorBottom;
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private boolean tempOverride = false;
    private boolean on = false;

    public Flywheel(Robot robot) {
        flywheelMotorTop = robot.hardwareMap.get(DcMotorEx.class, "flywheel");
//        flywheelMotorBottom = robot.hardwareMap.get(DcMotorEx.class, "flywheelBottom");
//        flywheelMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = robot.telemetry;
        drivetrain = robot.drivetrain;
    }

    public void setShootingVelocity(double shootingVelocity) {
        this.params.shootingVelocity = shootingVelocity;
        tempOverride = true;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.params.targetVelocity = targetVelocity;
        tempOverride = true;
    }

    public void useInterpolation() {
        tempOverride = false;
    }

    private void setPower(double power) {
        flywheelMotorTop.setPower(power);
//        flywheelMotorBottom.setPower(power);
    }

    private double getVelocity() {
        return -flywheelMotorTop.getVelocity();
    }

    public boolean atTarget() {
        return Math.abs(params.targetVelocity - getVelocity()) <= params.velocityTolerance;
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
                params.targetVelocity = params.shootingVelocity;
                setPower(params.kP * (params.targetVelocity - getVelocity()) + params.kV * params.targetVelocity + params.kS * Math.signum(params.targetVelocity));
            } else {
                params.targetVelocity = 0;
                setPower(0);
            }

            telemetry.addData("Flywheel Velocity", getVelocity());
            telemetry.addData("Flywheel Target", params.targetVelocity);
            telemetry.addData("Flywheel Power", flywheelMotorTop.getPower());
        });
    }
}