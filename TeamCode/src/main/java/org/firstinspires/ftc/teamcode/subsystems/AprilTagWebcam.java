package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static com.pedropathing.ivy.commands.Commands.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer; // DOESN'T exist D:
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private String targetClusterName = "RED AUDIENCE"; // TODO make this update either based on button or pos (probably button)

    private Telemetry telemetry;

    private ExposureControl exposureControl;
    private final boolean showFullTelemetry = true;

    public AprilTagWebcam(Robot robot) {

        // Create/declare/assign aprilTagProcessor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES) // consider changing to inches instead of cm?
                .build();

        // Create/declare/assign visionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();

//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        webcamName.getCameraCharacteristics()
//        Camera camera = hardwareMap.get(Camera.class, "Webcam 1");
//        exposureControl = camera.getControl(ExposureControl.class);

        builder.setCamera(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
//        exposureControl = visionPortal.getCameraControl(ExposureControl.class);

//        exposureControl.getExposure(TimeUnit.MILLISECONDS);

//        telemetry.addData("exposure (i think plz idk man)", exposureControl.getExposure(TimeUnit.MILLISECONDS));
        telemetry = robot.telemetry;
    }

    public Command periodic() {
        return infinite(() -> {
            update();
            if (showFullTelemetry) {
                for (AprilTagDetection tagDetection : detectedTags) {
                    displayDetectionTelemtry(tagDetection);
                }
            }
        });
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

//    public ExposureControl getExposureControl() {
//        return exposureControl;
//    }


// TODO fix this function
    public void displayDetectionTelemtry(AprilTagDetection detectedTag) {
        if (detectedTag == null) {return;}
        if (detectedTag instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detectedTag;

            telemetry.addLine(String.format(
                    "\n==== %s", cluster.metadata.name));

            telemetry.addLine(String.format(
                    "XYZ %6.1f %6.1f %6.1f (inch)",
                    detectedTag.ftcPose.x,
                    detectedTag.ftcPose.y,
                    detectedTag.ftcPose.z));

            telemetry.addLine(String.format(
                    "PRY %6.1f %6.1f %6.1f (deg)",
                    detectedTag.ftcPose.pitch,
                    detectedTag.ftcPose.roll,
                    detectedTag.ftcPose.yaw));

            telemetry.addLine(String.format(
                    "RBE %6.1f %6.1f %6.1f (inch, deg, deg)",
                    detectedTag.ftcPose.range,
                    detectedTag.ftcPose.bearing,
                    detectedTag.ftcPose.elevation));

            telemetry.addLine(String.format(
                    "Cluster Found: %d%%",
                    cluster.percentClusterFound));
        }
    }

    public AprilTagClusterDetection getClusterByName(String name) {

        if (name == null) {
            return null;
        }

        for (AprilTagDetection detection : detectedTags) {

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata.name.equalsIgnoreCase(name)) {
                    return cluster;
                }
            }
        }

        return null;
    }

    public List<AprilTagClusterDetection> getVisibleClusters() {

        List<AprilTagClusterDetection> clusters = new ArrayList<>();

        for (AprilTagDetection detection : detectedTags) {

            if (detection instanceof AprilTagClusterDetection) {
                clusters.add((AprilTagClusterDetection) detection);
            }
        }

        return clusters;
    }

    public AprilTagClusterDetection getTargetCluster() {
        return getClusterByName(targetClusterName);
    }

    public void setTargetCluster(String name) {
        targetClusterName = name;
    }

    /**
     * Gets name of target cluster. Returns null if no clusters detected
     * @return
     */
    public String getTargetName() {

        AprilTagClusterDetection target = getTargetCluster();

        if (target == null) {
            return null;
        }

        return target.metadata.name;
    }


    public double getTargetX() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.x : Double.NaN;
    }

    public double getTargetY() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.y : Double.NaN;
    }

    public double getTargetZ() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.z : Double.NaN;
    }

    public double getTargetPitch() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.pitch : Double.NaN;
    }

    public double getTargetRoll() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.roll : Double.NaN;
    }

    public double getTargetYaw() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.yaw : Double.NaN;
    }

    public double getTargetRange() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.range : Double.NaN;
    }

    public double getTargetBearing() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.bearing : Double.NaN;
    }

    public double getTargetElevation() {
        AprilTagClusterDetection target = getTargetCluster();
        return target != null ? target.ftcPose.elevation : Double.NaN;
    }

    public int getTargetPercentFound() {

        AprilTagClusterDetection target = getTargetCluster();

        return target != null ? target.percentClusterFound : 0;
    }

    public boolean isTargetUpright() {

        AprilTagClusterDetection target = getTargetCluster();

        if (target == null) {
            return false;
        }

        return Math.abs(target.ftcPose.roll) < 90;
    }

    public void stop() {
        if (visionPortal != null) { // if still an instance of visionPortal, then close
            visionPortal.close();
        }
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

//    public double getExposure() {
//        return exposureControl.getExposure(TimeUnit.MILLISECONDS);
//    }
}