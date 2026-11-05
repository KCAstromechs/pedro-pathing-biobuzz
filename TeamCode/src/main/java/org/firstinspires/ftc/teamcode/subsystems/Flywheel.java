package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.math.TurretLocation;
//import org.firstinspires.ftc.teamcode.math.WaveLength;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

@Config
public class Flywheel {

    public static class FlywheelParams {
        public double kP = 5.75;
        public double kF = 12.75;
        public int velocityTolerance = 40; //in encoder ticks/sec
        public double shootingVelocity = 1270; //in encoder ticks/sec
        private double targetVelocity = 0;
    }

    public static FlywheelParams params = new FlywheelParams();

    public static boolean override = false;
    private final DcMotorEx flywheelMotor;
//    private final DcMotorEx flywheelMotorBottom;
    private final Telemetry telemetry;
    private final Drivetrain drivetrain;
    private boolean tempOverride = false;
    private boolean on = false;
    private boolean reverse = false;

    public Flywheel(Robot robot) {
        flywheelMotor = robot.hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TODO confirm direction

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(params.kP, 0, 0, params.kF);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry = robot.telemetry;
        drivetrain = robot.drivetrain;
        telemetry.addLine("Initialization Complete");
    }

    public void setShootingVelocity(double shootingVelocity) {
        params.shootingVelocity = shootingVelocity;
        tempOverride = true;
    }

    public void setTargetVelocity(double targetVelocity) {
        params.targetVelocity = targetVelocity;
        tempOverride = true;
    }

    public void useInterpolation() {
        tempOverride = false;
    }

    /**
     * Sets desired flyWheel velocity to `params.targetVelocity`
     */
    public void updateVelocity() { flywheelMotor.setVelocity(params.targetVelocity); }

    /**
     * Return flywheel velocity in ticks per second
     * @return
     */
    private double getVelocity() {
        return flywheelMotor.getVelocity();
    }

    /**
     * Return whether flywheel is at shooting velocity with `params.velocityTolerance`
     * @return
     */
    public boolean atTarget() {
        return Math.abs(params.targetVelocity - getVelocity()) <= params.velocityTolerance && params.targetVelocity== params.shootingVelocity;
    }

    public void turnOn() {
        on = true;
    }

    public void turnOff() {
        on = false;
        reverse = false;
    }

    public void turnReverse(){reverse = true; on =false;}

    public Command off() {
        return instant(() -> turnOff()).requiring(flywheelMotor);
    }

    public Command on() {
        return instant(() -> turnOn()).requiring(flywheelMotor);
    }

    public Command reverse() {
        return instant(() -> turnReverse()).requiring(flywheelMotor);
    }

    public Command periodic() {
        return infinite(() -> {

            if (on) {
                setTargetVelocity(params.shootingVelocity);
            } else if (reverse) {
                setTargetVelocity(-500);
            }else {
                setTargetVelocity(0);
            }
            updateVelocity();

            telemetry.addLine("-----Flywheel Telemetry:");
            telemetry.addData("override: ", override);
            telemetry.addData("kP: ", params.kP);
            telemetry.addData("kF: ", params.kF);
            telemetry.addData("Flywheel Velocity (ticks per sec)", getVelocity());
            telemetry.addData("Flywheel Target (ticks per sec)", params.targetVelocity);
            telemetry.addData("Flywheel Power", flywheelMotor.getPower());
        });
    }
}