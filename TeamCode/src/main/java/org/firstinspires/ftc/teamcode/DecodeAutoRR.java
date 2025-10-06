package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.vision.opencv.ColorRange; // Assuming ColorRange.ARTIFACT_PURPLE is available
import org.firstinspires.ftc.vision.opencv.Circle; // Added import for Circle

import java.util.List;

/*
 * Autonomous OpMode for the 2025 Decode Season (Template)
 *
 * This OpMode:
 * 1. Uses ColorBlobLocatorProcessor to detect an "artifact" position (e.g., LEFT, CENTER, RIGHT).
 * 2. Uses AprilTagProcessor to detect AprilTags.
 * 3. Uses RoadRunner v1 (with Actions) to navigate based on the artifact to an AprilTag zone.
 *
 * **IMPORTANT:** This is a template. You MUST update:
 * - Artifact detection logic and zones (TARGET_X_LEFT_THRESHOLD, TARGET_X_RIGHT_THRESHOLD).
 * - RoadRunner trajectories (startPose, positions, headings).
 * - Target AprilTag IDs and logic for interacting with them.
 * - Ensure ColorRange.ARTIFACT_PURPLE is correctly defined and available.
 */
@Autonomous(name = "Decode Auto RR", group = "RoadRunner")
public class DecodeAutoRR extends LinearOpMode {

    // Vision variables
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor artifactDetector;
    private AprilTagProcessor aprilTagProcessor;

    // RoadRunner drive class
    private MecanumDrive drive; // Assuming your MecanumDrive class is named this and in this package

    // Enum for artifact position
    private enum ArtifactPosition {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }

    // --- TUNING CONSTANTS ---
    // **UPDATE THESE VALUES BASED ON THE GAME AND YOUR ROBOT/CAMERA SETUP**

    // Camera resolution for artifact detection
    private static final int CAMERA_WIDTH = 640; // pixels
    private static final int CAMERA_HEIGHT = 480; // pixels

    // Thresholds for determining artifact position based on X-coordinate
    // (assuming camera is centered and artifact appears in one of three zones)
    private static final double TARGET_X_LEFT_THRESHOLD = CAMERA_WIDTH / 3.0;
    private static final double TARGET_X_RIGHT_THRESHOLD = 2.0 * CAMERA_WIDTH / 3.0;

    // AprilTag IDs to look for (UPDATE THESE FOR THE GAME)
    private static final int LEFT_APRILTAG_ID = 1;   // Example ID if artifact is on the left
    private static final int CENTER_APRILTAG_ID = 2; // Example ID if artifact is on the center
    private static final int RIGHT_APRILTAG_ID = 3;  // Example ID if artifact is on the right
    private int targetAprilTagId = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        // --- INITIALIZATION ---

        // RoadRunner
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0)); // Adjust start pose as needed

        // Vision
        artifactDetector = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // Using predefined ARTIFACT_PURPLE
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // Added contour mode
                // Add other builder methods like setMinContourArea, setMaxContourArea, etc. from your AutoDriveToArtifact if needed
                .setDrawContours(true)
                .setBoxFitColor(Color.rgb(255,0,0))
                .setCircleFitColor(Color.rgb(0,255,0)) // This color is for the circle fit visualization
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // Common family, adjust if needed
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // Adjust units as needed
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Ensure your webcam name is correct
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(artifactDetector)
                .addProcessor(aprilTagProcessor)
                // .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // Can improve performance
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.addData("Artifact Color", "Using ColorRange.ARTIFACT_PURPLE");
        telemetry.update();

        // --- WAIT FOR START ---
        while (!isStarted() && !isStopRequested()) {
            // Update pose estimate before each trajectory
            drive.updatePoseEstimate(); // Important for accurate starting poses for actions
            ArtifactPosition detectedPosition = detectArtifactPosition();
            telemetry.addData("Detected Artifact Position", detectedPosition);

            // You can also show AprilTag detections if any are visible
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("  Tag ID %d visible", detection.id));
                }
            }
            telemetry.update();
            sleep(20); // Small delay to prevent busy-waiting
        }

        if (isStopRequested()) return;

        // --- AUTONOMOUS PERIOD ---
        // Update pose estimate before starting autonomous actions
        drive.updatePoseEstimate();

        // 1. Final artifact detection after start
        ArtifactPosition artifactPosition = detectArtifactPosition();
        telemetry.addData("Final Artifact Position", artifactPosition);
        telemetry.update();

        // Disable artifact detector if no longer needed to save resources
        // visionPortal.setProcessorEnabled(artifactDetector, false);

        // 2. Determine target AprilTag based on artifact position
        switch (artifactPosition) {
            case LEFT:
                targetAprilTagId = LEFT_APRILTAG_ID;
                break;
            case CENTER:
                targetAprilTagId = CENTER_APRILTAG_ID;
                break;
            case RIGHT:
                targetAprilTagId = RIGHT_APRILTAG_ID;
                break;
            case NOT_FOUND:
                // Handle case where artifact is not found - e.g., run a default path
                telemetry.addLine("Artifact not found, running default path or stopping.");
                telemetry.update();
                // Example: Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).lineToX(10).build());
                return; // Or implement a default routine
        }
        telemetry.addData("Target AprilTag ID", targetAprilTagId);
        telemetry.update();


        // 3. Define RoadRunner Actions to navigate
        // **THESE ARE PLACEHOLDER TRAJECTORIES - UPDATE THEM FOR THE ACTUAL GAME FIELD**

        // Update pose estimate before each trajectory
        drive.updatePoseEstimate();
        if (artifactPosition == ArtifactPosition.LEFT) {
            Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose()) // Corrected pose access
                    .lineToX(24) // Drive 24 inches forward
                    .turn(Math.toRadians(90)) // Turn 90 degrees left
                    .lineToX(12) // Drive another 12 inches towards the "left" AprilTag zone
                    .build()
            );
        } else if (artifactPosition == ArtifactPosition.CENTER) {
            // Update pose estimate before each trajectory
            drive.updatePoseEstimate();
            Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose()) // Corrected pose access
                    .lineToX(30) // Drive 30 inches forward towards "center" AprilTag zone
                    .build()
            );
        } else if (artifactPosition == ArtifactPosition.RIGHT) {
            // Update pose estimate before each trajectory
            drive.updatePoseEstimate();
            Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose()) // Corrected pose access
                    .lineToX(24) // Drive 24 inches forward
                    .turn(Math.toRadians(-90)) // Turn 90 degrees right
                    .lineToX(12) // Drive another 12 inches towards the "right" AprilTag zone
                    .build()
            );
        }

        telemetry.addLine("Navigation to AprilTag zone complete (simulated).");
        telemetry.update();
        
        // Update pose estimate before looking for AprilTags or doing more actions
        drive.updatePoseEstimate();

        // 4. Fine-tune approach using AprilTag (Optional and more advanced)
        long startTime = System.currentTimeMillis();
        boolean tagFound = false;
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 5000 && !tagFound) { // Look for 5 seconds
            drive.updatePoseEstimate(); // Keep pose updated while searching
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == targetAprilTagId) {
                    tagFound = true;
                    telemetry.addLine(String.format("\nFound Target AprilTag ID %d!", detection.id));
                    telemetry.addData("  Range", "%.2f inches", detection.ftcPose.range);
                    telemetry.addData("  Bearing", "%.2f degrees", detection.ftcPose.bearing);
                    telemetry.addData("  Yaw", "%.2f degrees", detection.ftcPose.yaw);
                    // Example of using detection data for a final move (requires drive.updatePoseEstimate() before actionBuilder)
                    // Pose2d currentPose = drive.localizer.getPose();
                    // double dx = detection.ftcPose.x; // Simplified, needs proper transform
                    // double dy = detection.ftcPose.y; // Simplified, needs proper transform
                    // Actions.runBlocking(drive.actionBuilder(currentPose).lineToX(currentPose.position.x + dx).strafeLeft(dy).build());
                    telemetry.update();
                    break; 
                }
            }
            if (!tagFound) {
                telemetry.addLine("Searching for AprilTag ID: " + targetAprilTagId);
                telemetry.update();
            }
            sleep(20);
        }

        if (!tagFound) {
            telemetry.addLine("Target AprilTag ID: " + targetAprilTagId + " not found after navigation.");
            telemetry.update();
        }

        // --- CLEANUP ---
        // visionPortal.close(); // Automatically closed when OpMode stops if setAutoStopLiveView(true)

        // Wait for the OpMode to end
        while (opModeIsActive()) {
            drive.updatePoseEstimate(); // Keep updating pose if doing anything in the loop
            idle();
        }
    }

    /**
     * Detects the position of the artifact using ColorBlobLocatorProcessor.
     * @return ArtifactPosition enum (LEFT, CENTER, RIGHT, NOT_FOUND)
     */
    private ArtifactPosition detectArtifactPosition() {
        List<ColorBlobLocatorProcessor.Blob> blobs = artifactDetector.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                100, 200000, blobs); // Adjust min/max area as needed

        if (!blobs.isEmpty()) {
            // Find the largest blob
            ColorBlobLocatorProcessor.Blob largestBlob = null;
            double maxArea = 0;
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                if (blob.getContourArea() > maxArea) { // Corrected: getContourArea()
                    maxArea = blob.getContourArea();    // Corrected: getContourArea()
                    largestBlob = blob;
                }
            }

            if (largestBlob != null) {
                Circle circleFit = largestBlob.getCircle();
                if (circleFit == null) { // Check if a circle fit was successful
                    telemetry.addLine("Could not fit circle to largest blob.");
                    return ArtifactPosition.NOT_FOUND;
                }
                double blobCenterX = circleFit.getX(); // Corrected: getX() from Circle

                telemetry.addData("Largest Blob X", "%.2f", blobCenterX);
                telemetry.addData("Largest Blob Area", "%.2f", largestBlob.getContourArea()); // Corrected: getContourArea()

                if (blobCenterX < TARGET_X_LEFT_THRESHOLD) {
                    return ArtifactPosition.LEFT;
                } else if (blobCenterX < TARGET_X_RIGHT_THRESHOLD) {
                    return ArtifactPosition.CENTER;
                } else {
                    return ArtifactPosition.RIGHT;
                }
            }
        }
        return ArtifactPosition.NOT_FOUND;
    }
}
