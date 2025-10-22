package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.ballShooter;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Red1Auto
 * - Start pose: (60, -12) heading 180°
 * - Uses Roadrunner MecanumDrive (your project)
 * - Uses VisionManager (multi-color) with single webcam
 */
@Autonomous(name = "Red Wall", group = "Auto")
public class Red1Auto extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(Red1Auto.class);
    private MecanumDrive drive;
    private VisionManager vision;
    private Intake intake = new Intake();
    private ballShooter shooter = new ballShooter();

    // Starting pose (in inches) and heading (radians)
    private final Pose2d startPose = new Pose2d(60, -12, Math.toRadians(180));

    // --- TUNING CONSTANTS ---
    // NOTE: VisionManager below is created with 1280x720 - center X = 640
    private static final int TARGET_X = 640;
    private static final double TURN_GAIN = 0.006; // tune on-field
    private static final double FORWARD_SPEED = 0.45; // safer default
    private static final int STOP_RADIUS = 80;

    private static final int DESIRED_TAG_ID = 24;   // Choose the tag you want to approach or set to -1 for ANY tag.
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    public class DriveToArtifact implements InstantFunction {
        @Override
        public void run() {
            if (vision == null) {
                telemetry.addLine("Vision missing");
                telemetry.update();
                return;
            }

            ColorBlobLocatorProcessor.Blob targetBlob = vision.largestBlob();
            if (targetBlob == null) {
                telemetry.addLine("No blob");
                telemetry.update();
                return;
            }

            Circle circleFit = targetBlob.getCircle();
            if (circleFit == null) {
                telemetry.addLine("No circle fit");
                telemetry.update();
                return;
            }

            // If close enough, stop
            if (circleFit.getRadius() > STOP_RADIUS) {
                // Stop robot and intake
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));
                try {
                    intake.stopIntake();
                } catch (Exception ignored) {
                }
                telemetry.addLine("Target reached");
                telemetry.update();
                return;
            }

            // Approach: forward + small angular correction based on horizontal error
            intake.runIntake();

            double error = TARGET_X - circleFit.getX(); // positive => target is left of center if x is measured from left
            double turn = error * TURN_GAIN;

            // clamp turn
            double maxTurn = 0.7;
            if (turn > maxTurn) turn = maxTurn;
            if (turn < -maxTurn) turn = -maxTurn;

            // setDrivePowers accepts PoseVelocity2d(linearVelVector, angularVel)
            // linear vector is (forward, lateral). We move forward along x (first component).
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(FORWARD_SPEED, 0.0), turn));

            telemetry.addLine("Driving to artifact");
            telemetry.addData("blobX", "%.1f", circleFit.getX());
            telemetry.addData("err", "%.1f", error);
            telemetry.addData("r", "%.1f", circleFit.getRadius());
            telemetry.addData("turn", "%.3f", turn);
            telemetry.update();
        }
    }

    public class Shooter implements InstantFunction {
        @Override
        public void run() {
            if (vision == null) {
                shooter.stop();
                return;
            }
            shooter.shoot(1.0);
//            if (vision.tagDesired(DESIRED_TAG_ID)) {
//                shooter.shoot(1.0);
//            } else {
//                shooter.stop();
//            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Init drive");
        telemetry.update();

        // Construct drive with start pose (constructor in your code takes HardwareMap & Pose2d)
        drive = new MecanumDrive(hardwareMap, startPose);

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        vision = new VisionManager(hardwareMap, cam, new Size(640, 480));

        // Ensure localizer has correct start pose
        try {
            drive.localizer.setPose(startPose);
        } catch (Exception e) {
            // If localizer API differs, ignore but log
            log.warn("Couldn't set localizer pose: " + e.getMessage());
        }

        drive.updatePoseEstimate(); // update once

        telemetry.addData("Pose", startPose);
        telemetry.update();

        // ---------- VISION ----------
        telemetry.addData("Vision", "Initializing...");
        telemetry.update();
        try {
            vision.startDashboardStream(15);
        } catch (Exception ignored) {
            telemetry.addData("Vision", "No worky");

        }

        // Initialize intake
        intake.init(hardwareMap);

        // Precompute example trajectories (tune values)
        double goalX = -30.0;    // TUNE
        double goalY = 30.0;     // TUNE
        double goalHeading = Math.toRadians(135);

        double parkX = 37.0;     // TUNE
        double parkY = -33.0;

        drive.updatePoseEstimate();

        TrajectoryActionBuilder toPark = drive.actionBuilder(startPose)
                .strafeToSplineHeading(new Vector2d(parkX, parkY), Math.toRadians(90));

        TrajectoryActionBuilder toGoal = drive.actionBuilder(startPose)
                .splineToSplineHeading(new Pose2d(goalX, goalY, goalHeading), Math.toRadians(90));

        Action toSpike1 = drive.actionBuilder(startPose)
                .splineToSplineHeading(new Pose2d(36, 30, Math.toRadians(90)), Math.toRadians(90))
                .stopAndAdd(new DriveToArtifact())
                .build();

        telemetry.addData("Status", "Init complete. Waiting...");
        telemetry.update();

        // allow small sampling during init
        while (!isStarted() && !isStopRequested()) {
            if (vision != null && vision.largestBlob() != null) {
                Circle c = vision.largestBlob().getCircle();
                if (c != null) {
                    telemetry.addData("init blob x", "%.1f", c.getX());
                    telemetry.addData("init blob r", "%.1f", c.getRadius());
                }
            }
            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) return;

        telemetry.addData("AutoStep", "Running sequence");
        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                toSpike1,
                toGoal.build(),
                toPark.build()
        ));
    }
}
