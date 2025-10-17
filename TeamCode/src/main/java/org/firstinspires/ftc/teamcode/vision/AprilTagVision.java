package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

import java.util.List;

public class AprilTagVision {
    private final AprilTagProcessor tagProcessor;
    private final VisionPortal visionPortal;

    public AprilTagVision(HardwareMap hardwareMap) {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(BuiltinCameraDirection.BACK)
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        return tagProcessor.getDetections();
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
