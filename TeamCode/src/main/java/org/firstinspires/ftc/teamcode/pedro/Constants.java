package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.596677885280819); // manual: -7.184
        c.yPodOffset.set(-7.4570020540492745); // manual: 7.217
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.20295588372940623);
                Controller secondaryTranslationalForward = Controller.proportional(0.07498677595692405);
                Controller primaryTranslationalLateral = Controller.proportional(0.2700768513270842);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09978617997898039);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016288227851107805));
                c.brake.set(Controller.proportionalFeedforward(0.013844993673441634));

                c.headingFeedback.set(Controller.proportional(3.6569717283769325));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05289465019832839, 0.0031321382416240376));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07460235979226991, 0.05747613265885763));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.659647418843692E-4, 0.0010560744996894586));

                c.maxAchievableForwardVelocity.set(63.36411943976557);
                c.maxAchievableStrafeVelocity.set(55.236670226807824);
                c.naturalForwardDeceleration.set(32.077292921871106);
                c.naturalStrafeDeceleration.set(57.81980441879844);
            }
    );
}