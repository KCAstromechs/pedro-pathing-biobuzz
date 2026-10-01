package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(config -> {
        config.frontRightName.set("frontRight");
        config.backRightName.set("backRight");
        config.backLeftName.set("backLeft");
        config.frontLeftName.set("frontLeft");

        // TODO change directions as needed
        config.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        config.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(config -> {
        // hardwareMapName -> name; hardware-map string is unchanged
        config.name.set("pinpoint");

        // offsets for pods
        config.xPodOffset.set(-7.184);
        config.yPodOffset.set(7.217);

        // distanceUnit(INCH) -> explicit offset and reported-position units.
        config.offsetUnits.set(DistanceUnit.INCH);
        config.globalDistanceUnit.set(DistanceUnit.INCH);

        // encoderResolution(enum) -> podType; original four-bar pods retained.
        config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // forwardEncoderDirection/strafeEncoderDirection -> x/yPodDirection.
        // TODO change directions as needed
        config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(config -> {
        config.maxAchievableForwardVelocity.set(74.0662); // TODO change for tuning
        config.maxAchievableStrafeVelocity.set(55.7839); // TODO change for tuning

        // SPACE FOR PEDRO 3 AUTOTUNE settings

        // Original: new PathConstraints(0.99, 100, 1, 1)... whatever that means
        config.parametricTConstraint.set(0.01); // 1 - 0.99
        config.timeoutConstraint.set(100.0); // Milliseconds, unchanged

        // MORE SPACE FOR PEDRO 3 AUTOTUNE settings
        config.maxAchievableForwardVelocity.set(50.0);
        config.maxAchievableStrafeVelocity.set(50.0);

        config.naturalForwardDeceleration.set(50.0);
        config.naturalStrafeDeceleration.set(50.0);
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}