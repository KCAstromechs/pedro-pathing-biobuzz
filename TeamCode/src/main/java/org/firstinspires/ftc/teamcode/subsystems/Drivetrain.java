package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Config
public class Drivetrain {
    public static double gateOpenHeadingDegrees = 36.5;
    private static Pose poseTransfer = Pose.zero();
    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final Follower follower;
    private final Telemetry telemetry;
    public static double headingP = 1.75;
    public static double headingI = 0;
    public static double headingD = 0.09;
    private final PIDController headingController = new PIDController(headingP, headingI, headingD);
    private boolean lockHeading = false;
    private double headingTargetRadians = 0;
    private double headingOffset = 0;

    public Drivetrain(Robot robot) {
        follower = Constants.createFollower(robot.hardwareMap);
        frontLeft = robot.hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = robot.hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = robot.hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = robot.hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = robot.telemetry;
    }

    private static double signedSquare(double raw) {
        return Math.signum(raw) * Math.pow(raw, 2);
    }

    public static void localize(Pose pose) {
        poseTransfer = pose;
    }

    public void gateHeading(Alliance alliance) {
        lockHeading = true;
        if (alliance == Alliance.RED) {
            headingTargetRadians = Math.toRadians(gateOpenHeadingDegrees);
        } else {
            headingTargetRadians = Math.toRadians(Math.PI - gateOpenHeadingDegrees);
        }
    }

    public void unlockHeading() {
        lockHeading = false;
    }

    /**
     * Resets the field-centric forward direction to the robot's current heading
     * while preserving its absolute (x, y) coordinates.
     */
    public void resetFieldCentricHeading() {
        this.headingOffset = follower.getHeading();
    }

    /**
     * Resets the heading offset back to 0 (aligned with true field zero).
     */
    public void clearHeadingOffset() {
        this.headingOffset = 0.0;
    }

    public void arcadeDrive(double forward, double strafe, double turn, Alliance alliance) {
        // double headingRadians = follower.pose().heading(); // was goofy/didn't work for us
        double rawHeading = follower.pose().heading() - Math.PI / 2; // 
        double headingRadians = AngleUnit.normalizeRadians(rawHeading - headingOffset);

        if (lockHeading) {
            headingController.kP = headingP;
            headingController.kI = headingI;
            headingController.kD = headingD;

            double headingError = AngleUnit.normalizeRadians(
                    headingTargetRadians - rawHeading
            );

            turn = -headingController.calculate(
                    headingTargetRadians,
                    headingError
            );
        } else {
            turn = signedSquare(turn);
        }


        if (alliance == Alliance.BLUE) headingRadians += Math.PI;

        double x = strafe * Math.cos(headingRadians) + forward * Math.sin(headingRadians);
        double y = strafe * -Math.sin(headingRadians) + forward * Math.cos(headingRadians);
//        double x = strafe;
//        double y = forward;
        y *= 1.1;

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(turn), 1);

        frontLeft.setPower((y - x + turn) / denominator);
        frontRight.setPower((y + x + turn) / denominator);
        backLeft.setPower((y + x - turn) / denominator);
        backRight.setPower((y - x - turn) / denominator);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void setStartingPose(Pose pose) {
        follower.setPose(pose);
    }

    public void usePreviousStartingPose() {
        setStartingPose(poseTransfer);
    }

    public Command resetGyro() {
        return instant(() -> {
            resetFieldCentricHeading();
        });
    }

    public Command unResetGyro() {
        return instant(() -> {
            clearHeadingOffset();
        });
    }

    public Command followPath(Path path) {
        return follow(follower, path);
    }

    public Command periodic() {
        return infinite(() -> {
            follower.update();
            poseTransfer = follower.pose();

            telemetry.addData("Current X", follower.pose().x());
            telemetry.addData("Current Y", follower.pose().y());
            telemetry.addData("Current Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Heading Locked", lockHeading);
            telemetry.addData("Heading Target", headingTargetRadians);
        });
    }
}