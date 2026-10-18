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
        public double kP = 0.01;
        public double kS = 0.065;
        public double kV = 0.000365;
        public int velocityTolerance = 25;
        public double shootingVelocity = 1000; //in encoder ticks/sec
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

    /**
     * Return flywheel velocity in RPM
     * @return
     */
    private double getVelocity() {
        return flywheelMotorTop.getVelocity() / 6;
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
            double pidCalc = 0;
            if (on) {
//                target = override ? overrideTarget : (tempOverride ? target : WaveLength.getVelocityWithInterpolation(TurretLocation.getTurretPose(drivetrain.getPose()), Alliance.current));
                params.targetVelocity = params.shootingVelocity;
                pidCalc = params.kP * (params.targetVelocity - getVelocity()) + params.kV * params.targetVelocity + params.kS * Math.signum(params.targetVelocity);
                setPower(pidCalc);
            } else {
                params.targetVelocity = 0;
                setPower(0);
            }

            telemetry.addLine("-----Flywheel Telemetry:");
            telemetry.addData("override: ", override);
            telemetry.addData("kP: ", params.kP);
            telemetry.addData("kS: ", params.kS);
            telemetry.addData("kV: ", params.kV);
            telemetry.addData("PID Calculation: ", pidCalc);
            telemetry.addData("Flywheel Velocity (RPM)", getVelocity());
            telemetry.addData("Flywheel Target (RPM)", params.targetVelocity);
            telemetry.addData("Flywheel Power", flywheelMotorTop.getPower());
        });
    }
}