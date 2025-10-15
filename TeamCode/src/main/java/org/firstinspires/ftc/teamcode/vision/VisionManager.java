package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
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

    private VisionPortal portal;
    private final ColorBlobLocatorProcessor purpleProcessor;
    private final ColorBlobLocatorProcessor greenProcessor;
    private final Size resolution;
    private final WebcamName camera;

    public VisionManager(HardwareMap hardwareMap, WebcamName camera, Size res) {
        this.camera = camera;
        this.resolution = res;

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

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, camera.getDeviceName()))
                .setCameraResolution(resolution)
                .addProcessor(purpleProcessor)
                .addProcessor(greenProcessor)
                .addProcessor(tagProcessor)
                .build();
    }

    /** Start Dashboard camera stream (optional) */
    public void startDashboardStream(int fps) {
        try {
            com.acmerobotics.dashboard.FtcDashboard.getInstance().startCameraStream(portal, fps);
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

    /** Access VisionPortal (if you want to add more processors later) */
    public VisionPortal getPortal() { return portal; }

}
