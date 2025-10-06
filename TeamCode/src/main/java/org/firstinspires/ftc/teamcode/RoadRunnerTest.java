package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;

@Autonomous(name = "RoadRunner Test Auto", group = "RoadRunner")
public class RoadRunnerTest extends LinearOpMode {
    // Vision variables
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor artifactDetector;
    private AprilTagProcessor aprilTagProcessor;

    // RoadRunner drive class
    private MecanumDrive drive;

    private enum ArtifactPosition {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }

    // Camera resolution for artifact detection
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;
    private static final double TARGET_X_LEFT_THRESHOLD = CAMERA_WIDTH / 3.0;
    private static final double TARGET_X_RIGHT_THRESHOLD = 2.0 * CAMERA_WIDTH / 3.0;

    // AprilTag IDs to look for
    private static final int LEFT_APRILTAG_ID = 1;
    private static final int CENTER_APRILTAG_ID = 2;
    private static final int RIGHT_APRILTAG_ID = 3;
    private int targetAprilTagId = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        // RoadRunner
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Vision
        artifactDetector = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBoxFitColor(Color.rgb(255, 0, 0))
                .setCircleFitColor(Color.rgb(0, 255, 0))
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(artifactDetector)
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .build();

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // Detect artifact position
        ArtifactPosition artifactPosition = detectArtifactPosition();
        telemetry.addData("Artifact Position", artifactPosition);
        telemetry.update();

        // Detect AprilTag
        targetAprilTagId = getTargetAprilTagId(artifactPosition);
        AprilTagDetection tag = getAprilTagDetection(targetAprilTagId);
        if (tag != null) {
            telemetry.addData("AprilTag Found", tag.id);
            telemetry.addData("Tag X", tag.ftcPose.x);
            telemetry.addData("Tag Y", tag.ftcPose.y);
        } else {
            telemetry.addLine("No AprilTag detected");
        }
        telemetry.update();

        // Drive to artifact zone (example positions)
        Pose2d targetPose = getTargetPose(artifactPosition);
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeTo(targetPose.position)
                        .turnTo(targetPose.heading)
                        .build()
        );

        // Optionally drive to AprilTag if detected
        if (tag != null) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(tag.ftcPose.x, tag.ftcPose.y))
                            .build()
            );
        }
    }

    private ArtifactPosition detectArtifactPosition() {
        List<ColorBlobLocatorProcessor.Blob> blobs = artifactDetector.getBlobs();
        if (blobs == null || blobs.isEmpty()) return ArtifactPosition.NOT_FOUND;
        double x = blobs.get(0).getCircle().getX();
        if (x < TARGET_X_LEFT_THRESHOLD) return ArtifactPosition.LEFT;
        if (x < TARGET_X_RIGHT_THRESHOLD) return ArtifactPosition.CENTER;
        return ArtifactPosition.RIGHT;
    }

    private int getTargetAprilTagId(ArtifactPosition pos) {
        switch (pos) {
            case LEFT: return LEFT_APRILTAG_ID;
            case CENTER: return CENTER_APRILTAG_ID;
            case RIGHT: return RIGHT_APRILTAG_ID;
            default: return -1;
        }
    }

    private AprilTagDetection getAprilTagDetection(int id) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null) return null;
        for (AprilTagDetection tag : detections) {
            if (tag.id == id) return tag;
        }
        return null;
    }

    private Pose2d getTargetPose(ArtifactPosition pos) {
        switch (pos) {
            case LEFT: return new Pose2d(12, 36, Math.toRadians(0));
            case CENTER: return new Pose2d(24, 36, Math.toRadians(0));
            case RIGHT: return new Pose2d(36, 36, Math.toRadians(0));
            default: return new Pose2d(0, 0, 0);
        }
    }
}
