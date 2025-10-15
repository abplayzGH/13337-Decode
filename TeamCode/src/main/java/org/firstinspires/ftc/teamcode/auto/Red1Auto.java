package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.VisionManager;

/**
 * Red1Auto
 * - Start pose: (60, -12) heading 0°
 * - Uses Roadrunner sample-style MecanumDrive
 * - Uses single webcam VisionManager (color-dominant fallback)
 * - Intake/fire/gate ignored
 */
@Autonomous(name = "Red1Auto", group = "Auto")
public class Red1Auto extends LinearOpMode {

    private MecanumDrive drive;
    private VisionManager vision;

    // Starting pose (in inches) and heading (radians)
    private final Pose2d startPose = new Pose2d(60.0, -12.0, Math.toRadians(3*Math.PI/2));

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing drive...");
        telemetry.update();

        // ---------- DRIVE ----------
        // If your MecanumDrive constructor accepts only HardwareMap (sample style):
        drive = new MecanumDrive(hardwareMap, startPose);

        // If your MecanumDrive expects a start pose in ctor, replace the line above accordingly.
        // Ensure Roadrunner pose estimate is set to the start pose we intend to place the robot at:
        drive.updatePoseEstimate();

        telemetry.addData("Pose", startPose);
        telemetry.update();

        // ---------- VISION ----------
        telemetry.addData("Vision", "Initializing...");
        telemetry.update();

        try {
            // Use the hardware-config name for your webcam (default: "Webcam 1")
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
            vision = new VisionManager(hardwareMap, webcamName, new Size(640, 480));
            // optional dashboard stream
            try {
                vision.startDashboardStream(10);
            } catch (Exception ignored) {
            }
        } catch (Exception e) {
            telemetry.addData("Vision Init Error", e.getMessage());
            telemetry.update();
            vision = null;
        }

        telemetry.addData("Status", "Init complete. Waiting for start...");
        telemetry.update();

        // ---------- SAMPLE VISION DURING INIT ----------
        String sampledColor = "NONE";
        long sampleStart = System.currentTimeMillis();
        long sampleTimeoutMs = 800; // ms to sample in init

        while (!isStarted() && !isStopRequested()) {
            if (vision != null && System.currentTimeMillis() - sampleStart < sampleTimeoutMs) {
                try {
                    String dom = vision.getDominantColor();
                    if (dom != null && !dom.equals("NONE")) sampledColor = dom;
                    telemetry.addData("Vision Sample", dom);
                } catch (Exception ex) {
                    telemetry.addData("Vision Sample Error", ex.getMessage());
                }
            } else {
                telemetry.addData("Vision Sample", "skipped/timeout");
            }
            telemetry.addData("SampledColor", sampledColor);
            telemetry.addData("StartingPose", startPose);
            telemetry.update();

            sleep(50);
        }

        if (isStopRequested()) {
            return;
        }

        // ---------- MOTIF FALLBACK (color-based) ----------
        String motif;
        if ("GREEN".equals(sampledColor)) motif = "GPP";
        else if ("PURPLE".equals(sampledColor)) motif = "PPG";
        else motif = "GPP"; // default fallback

        telemetry.addData("Starting Auto", true);
        telemetry.addData("SampledColor", sampledColor);
        telemetry.addData("UsingMotifFallback", motif);
        telemetry.update();

        // ---------- TRAJECTORY PLAN (tune values) ----------
        // Tune these to your practice field target points.
        double goalX = 40.0;    // example, TUNE
        double goalY = -40.0;   // example, TUNE
        double goalHeading = Math.toRadians(5*Math.PI/4);

        double parkX = 20.0;    // example, TUNE to ensure leaving launch line
        double parkY = -12.0;

        // Ensure pose estimate is current
        drive.updatePoseEstimate();

        TrajectoryActionBuilder toPark = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(parkX, parkY));

        TrajectoryActionBuilder toGoal = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(goalX, goalY), goalHeading);

        Actions.runBlocking(
                new SequentialAction(
                        toGoal.build()
//                        toPark.build()
                ));
        // ---------- EXECUTE: toGoal ----------
        telemetry.addData("AutoStep", "Driving to goal approach...");
    }}