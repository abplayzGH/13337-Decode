package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//TODO Make this code actually work lol


@Autonomous(name = "Start at Red Goal", group = "Auto")
public class RedGoal extends LinearOpMode {

    private MecanumDrive drive;
    private VisionManager vision;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;

    private final Pose2d startPose = new Pose2d(60, -12, Math.toRadians(180));

    private static final int[] TARGET_TAGS = {20, 24};

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- Init Subsystems ----------------
        drive = new MecanumDrive(hardwareMap, startPose);
        intake = new Intake(); intake.init(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        turret = new Turret(); turret.init(hardwareMap, "turret_motor");

        Servo latch = hardwareMap.get(Servo.class, "latchServo");


        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        vision = new VisionManager(hardwareMap, cam, new Size(640, 480));
        vision.startDashboardStream(15);

        drive.updatePoseEstimate();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // ---------------- Prebuilt Trajectories ----------------
        TrajectoryActionBuilder toFirstGoal = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(36, 30), Math.toRadians(90));

        TrajectoryActionBuilder parkTrajectory = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(37, -33));

        // ---------------- Run Auto ----------------
        Actions.runBlocking(new SequentialAction(
                toFirstGoal.build()
        ));

        while (opModeIsActive()) {

            drive.updatePoseEstimate();

            // ---------------- Vision ----------------
            AprilTagDetection target = null;
            for (int id : TARGET_TAGS) {
                target = vision.getTargetDetection(id);
                if (target != null) break;
            }

            // ---------------- Shooter ----------------
            if (target != null) {
                shooter.setMode(Shooter.Mode.DYNAMIC);
                if (shooter.isAtTargetVelocity()) {
                    latch.setPosition(0.3);
                    intake.runIntake();
                }
            } else {
                shooter.setRaw(0);
                intake.stopIntake();
            }

            // ---------------- Turret ----------------
            if (target != null) {
                turret.updateTurretTracking(target, getRuntime());
            } else {
                turret.setManualPower(0);
            }

            // ---------------- Update Drive ----------------
            drive.updatePoseEstimate();
        }

        // ---------------- Go to Park ----------------
        Actions.runBlocking(new SequentialAction(
                parkTrajectory.build()
        ));
    }
}
