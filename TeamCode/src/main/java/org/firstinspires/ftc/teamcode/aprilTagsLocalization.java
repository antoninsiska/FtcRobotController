package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * FTC DECODE 2024/25: Estimate robot field pose using AprilTags
 * - Works with a webcam "Webcam 1"
 * - Field size: 12 ft x 12 ft = 3.6576 m x 3.6576 m
 * - Tags: ID 20, 21, 22, 33, 24
 */
@TeleOp(name = "AT Field Localization (DECODE)", group = "Vision")
public class aprilTagsLocalization extends LinearOpMode {

    private static final String WEBCAM_NAME = "Webcam 1";

    // Physical tag size (outer black square) in meters
    private static final double TAG_SIZE_M = 0.0508; // 2 inches

    // Camera offset relative to robot reference point (meters, radians)
    // If your camera is exactly in the robot center and facing forward, keep as 0
    private static final double CAM_OFF_X = 0.00; // +right
    private static final double CAM_OFF_Y = 0.00; // +forward
    private static final double CAM_OFF_H = 0.00; // +CCW

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /** Simple pose struct */
    private static class Pose { double x,y,h; Pose(double x,double y,double h){this.x=x;this.y=y;this.h=h;} }

    /** Map: Tag ID -> Field pose (meters, radians) */
    private static final Map<Integer, Pose> FIELD_TAGS = new HashMap<>();
    static {
        // Field size
        final double FIELD_SIZE = 3.6576;  // 12 ft in meters
        final double HALF = FIELD_SIZE / 2.0; // ~1.8288 m

        // FTC DECODE tag locations
        FIELD_TAGS.put(20, new Pose(0.0, FIELD_SIZE, Math.toRadians(0)));      // Top left (blue side), facing +X
        FIELD_TAGS.put(21, new Pose(HALF - 0.15, FIELD_SIZE, Math.toRadians(-90))); // Center panel left, facing -Y
        FIELD_TAGS.put(22, new Pose(HALF,          FIELD_SIZE, Math.toRadians(-90))); // Center panel middle, facing -Y
        FIELD_TAGS.put(33, new Pose(HALF + 0.15, FIELD_SIZE, Math.toRadians(-90))); // Center panel right, facing -Y
        FIELD_TAGS.put(24, new Pose(FIELD_SIZE, FIELD_SIZE, Math.toRadians(180)));  // Top right (red side), facing -X
    }

    @Override
    public void runOpMode() {

        // --- Build AprilTag library (must include size + DistanceUnit) ---
        AprilTagLibrary.Builder libBuilder = new AprilTagLibrary.Builder();
        for (Integer id : FIELD_TAGS.keySet()) {
            AprilTagMetadata meta = new AprilTagMetadata(id, "FTag_"+id, TAG_SIZE_M, DistanceUnit.METER);
            libBuilder.addTag(meta);
        }
        AprilTagLibrary lib = libBuilder.build();

        // --- AprilTag processor (draw annotations in DS INIT view) ---
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(lib)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // --- VisionPortal (MJPEG for faster INIT preview) ---
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        telemetry.addLine("INIT: Point the webcam at an AprilTag (Camera Stream visible in DS INIT).");
        telemetry.update();

        while (opModeInInit()) {
            showDetections();
            telemetry.update();
            idle();
        }

        waitForStart();

        while (opModeIsActive()) {
            Pose robot = estimateRobotFieldPose(); // rough estimate from closest tag

            if (robot != null) {
                telemetry.addData("Robot Field Pose", "x=%.3f m, y=%.3f m, h=%.1f deg",
                        robot.x, robot.y, Math.toDegrees(robot.h));
            } else {
                telemetry.addLine("Robot Field Pose: (no known tag visible)");
            }

            showDetections();
            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    /** Show raw detections (IDs, range, bearing) */
    @SuppressLint("DefaultLocale")
    private void showDetections() {
        List<AprilTagDetection> ds = aprilTag.getDetections();
        telemetry.addData("Detections", ds.size());
        int i = 0;
        for (AprilTagDetection d : ds) {
            telemetry.addData("ID", "#%d  %d", i, d.id);
            if (d.ftcPose != null) {
                telemetry.addData("  range (m)", "%.2f", d.ftcPose.range / 100.0);
                telemetry.addData("  bearing (deg)", "%.1f", d.ftcPose.bearing);
            } else {
                telemetry.addLine("  (ftcPose=null — check tag library/size)");
            }
            i++;
        }
    }

    /**
     * Estimate rough 2D robot global pose from one visible AprilTag.
     * Assumptions:
     * - Tag field position and orientation are known (FIELD_TAGS).
     * - Camera is roughly facing the tag, so range is along tag's normal.
     * - Heading of the robot ~ tag heading (can be refined with yaw/bearing).
     */
    private Pose estimateRobotFieldPose() {
        List<AprilTagDetection> ds = aprilTag.getDetections();

        double bestRange = Double.POSITIVE_INFINITY;
        AprilTagDetection best = null;

        for (AprilTagDetection d : ds) {
            if (d.ftcPose == null) continue;
            if (!FIELD_TAGS.containsKey(d.id)) continue;
            double r = d.ftcPose.range / 100.0; // cm -> m
            if (r < bestRange) {
                bestRange = r;
                best = d;
            }
        }
        if (best == null) return null;

        Pose tag = FIELD_TAGS.get(best.id);
        double tagH = tag.h;

        // Tag normal vector in field frame
        double nx = Math.cos(tagH);
        double ny = Math.sin(tagH);

        // Camera position = tag position - range * normal
        double camX = tag.x - bestRange * nx;
        double camY = tag.y - bestRange * ny;

        // Approximate camera heading ~ tag heading
        double camH = tagH;

        // Correct for camera offset -> robot
        double cos = Math.cos(camH), sin = Math.sin(camH);
        double robX = camX - ( CAM_OFF_X * cos - CAM_OFF_Y * sin );
        double robY = camY - ( CAM_OFF_X * sin + CAM_OFF_Y * cos );
        double robH = normalize(camH + CAM_OFF_H);

        return new Pose(robX, robY, robH);
    }

    private static double normalize(double a){
        while (a >  Math.PI) a -= 2*Math.PI;
        while (a <= -Math.PI) a += 2*Math.PI;
        return a;
    }
}