// java
// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/VisionManager.java
package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

/**
 * MultiColorVisionManager:
 * Handles TWO color detectors (e.g., ARTIFACT_PURPLE and ARTIFACT_GREEN)
 * using a single VisionPortal and webcam.
 */
public class VisionManager {

    private final VisionPortal portal;
    private final ColorBlobLocatorProcessor purpleProcessor;
    private final ColorBlobLocatorProcessor greenProcessor;
    private final AprilTagProcessor tagProcessor; // field to hold processor

    public VisionManager(HardwareMap hardwareMap, WebcamName camera, Size res) {

        // --- PURPLE DETECTOR ---
        purpleProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .build();

        // --- GREEN DETECTOR ---
        greenProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setCircleFitColor(Color.rgb(0, 255, 0))
                .build();

        // assign to the field (was a local variable previously)
        this.tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(res)
                .addProcessor(purpleProcessor)
                .addProcessor(greenProcessor)
                .addProcessor(this.tagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }
    // CORRECTION

    /** Start Dashboard camera stream (optional) */
    public void startDashboardStream(int fps) {
        try {
            FtcDashboard.getInstance().startCameraStream(portal, fps);
        } catch (Exception ignored) {}
    }

    /** Stop vision portal */
    public void stop() {
        if (portal != null) portal.close();
    }

    /** Get list of purple blobs */
    public List<ColorBlobLocatorProcessor.Blob> getPurpleBlobs() {
        List<ColorBlobLocatorProcessor.Blob> blobs = purpleProcessor.getBlobs();
        if (blobs == null) return new ArrayList<>();
        return new ArrayList<>(blobs);
    }

    /** Get list of green blobs */
    public List<ColorBlobLocatorProcessor.Blob> getGreenBlobs() {
        List<ColorBlobLocatorProcessor.Blob> blobs = greenProcessor.getBlobs();
        if (blobs == null) return new ArrayList<>();
        return new ArrayList<>(blobs);
    }

    /** Determine which color is more dominant (larger blob area) */
    public String getDominantColor() {
        List<ColorBlobLocatorProcessor.Blob> purple = getPurpleBlobs();
        List<ColorBlobLocatorProcessor.Blob> green = getGreenBlobs();

        ColorBlobLocatorProcessor.Blob bestPurple = BlobUtils.largestBlob(purple);
        ColorBlobLocatorProcessor.Blob bestGreen = BlobUtils.largestBlob(green);

        double purpleArea = (bestPurple != null) ? bestPurple.getContourArea() : 0;
        double greenArea = (bestGreen != null) ? bestGreen.getContourArea() : 0;

        if (purpleArea > greenArea && purpleArea > 0) return "PURPLE";
        if (greenArea > purpleArea && greenArea > 0) return "GREEN";
        return "NONE";
    }

    public ColorBlobLocatorProcessor.Blob largestBlob(){
        List<ColorBlobLocatorProcessor.Blob> purple = getPurpleBlobs();
        List<ColorBlobLocatorProcessor.Blob> green = getGreenBlobs();

        ColorBlobLocatorProcessor.Blob bestPurple = BlobUtils.largestBlob(purple);
        ColorBlobLocatorProcessor.Blob bestGreen = BlobUtils.largestBlob(green);

        double purpleArea = (bestPurple != null) ? bestPurple.getContourArea() : 0;
        double greenArea = (bestGreen != null) ? bestGreen.getContourArea() : 0;

        if (purpleArea > greenArea && purpleArea > 0) return bestPurple;
        if (greenArea > purpleArea && greenArea > 0) return bestGreen;
        return null;
    }

    /** Access VisionPortal (if you want to add more processors later) */
    public VisionPortal getPortal() { return portal; }

    public List<AprilTagDetection> getDetections() {
        // guard: return empty list if tagProcessor wasn't created for any reason
        if (tagProcessor == null) return new ArrayList<>();
        List<AprilTagDetection> dets = tagProcessor.getDetections();
        if (dets == null) return new ArrayList<>();
        return dets;
    }

    // java
// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/VisionManager.java
// ... (existing code, imports, class declaration)

// ... (existing methods: getPurpleBlobs, getGreenBlobs, getDominantColor, largestBlob, getPortal, getDetections, tagDesired)

    /**
     * Retrieves the AprilTagDetection object for a specific tag ID.
     * @param targetTagId The ID of the AprilTag to find.
     * @return The detected AprilTag or null if not found.
     */
    public AprilTagDetection getTargetDetection(int targetTagId) {
        List<AprilTagDetection> detections = getDetections();
        if (detections.isEmpty()) {
            return null;
        }

        for (AprilTagDetection detection : detections) {
            // Check if the detection ID matches the target ID
            if (detection.id == targetTagId) {
                // Ensure the tag has valid pose data (metadata must be present for pose)
                if (detection.metadata != null && detection.ftcPose != null) {
                    return detection;
                }
            }
        }
        return null;
    }

// } // end of VisionManager class

    /**
     * Returns true if a detection matching `id` exists.
     * If id < 0, treat as wildcard: return true if any valid tag (with metadata) is present.
     */
    public boolean tagDesired(int id) {
        List<AprilTagDetection> dets = getDetections();
        if (dets.isEmpty()) return false;

        if (id < 0) {
            for (AprilTagDetection detection : dets) {
                if (detection.metadata != null) return true;
            }
            return false;
        }

        for (AprilTagDetection detection : dets) {
            if (detection.id == id) {
                return true;
            }
        }
        return false;
    }

}
