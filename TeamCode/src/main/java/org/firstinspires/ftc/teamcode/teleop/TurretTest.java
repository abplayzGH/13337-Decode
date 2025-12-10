package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Turret; // Import the new subsystem
import org.firstinspires.ftc.teamcode.vision.VisionManager; // Import the vision manager
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Reusable Turret Tracking", group = "Final Demo")
public class TurretTest extends LinearOpMode {

    // --- REUSABLE COMPONENTS ---
    private VisionManager visionManager = null;
    private Turret turretSubsystem = null;

    // --- CONFIGURATION ---
    private static final int TARGET_TAG_ID = 21;
    private static final String CAMERA_NAME = "Webcam 1";
    private static final String MOTOR_NAME = "turret_motor";

    @Override
    public void runOpMode() {
        // 1. Initialize Subsystems and Managers
        turretSubsystem = new Turret();
        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionManager = new VisionManager(hardwareMap, cam , new Size(640, 480));
        turretSubsystem.TurretSubsystem(hardwareMap, MOTOR_NAME);

        // 2. Wait for Start
//        telemetry.addData("Vision Status", visionManager.getStatus());
        telemetry.addData("Turret Status", "Ready to Track Tag %d", TARGET_TAG_ID);
        telemetry.update();

        waitForStart();

        // 3. Main Control Loop
        while (opModeIsActive()) {
            // A. Get the necessary data from the VisionManager
            AprilTagDetection detection = visionManager.getTargetDetection(21);

            // B. Pass the data to the TurretSubsystem for processing
            double motorPower = turretSubsystem.updateTurretTracking(detection, getRuntime());

            // C. Telemetry (displaying status)
            if (detection != null) {
                telemetry.addData("Status", "Tracking Tag %d", TARGET_TAG_ID);
                telemetry.addData("Bearing Error", "%.2f", 0.0 - detection.ftcPose.bearing); // Target - Current
            } else {
                telemetry.addData("Status", "Searching for Tag %d...", TARGET_TAG_ID);
            }
            telemetry.addData("Motor Power", "%.2f", motorPower);
            telemetry.update();
        }

        // 4. Cleanup
        turretSubsystem.stop();
        visionManager.stop();
    }
}