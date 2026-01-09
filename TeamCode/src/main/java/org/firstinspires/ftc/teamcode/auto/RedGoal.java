package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Color;
import android.util.Size;
import androidx.annotation.NonNull;


// RR-specific imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    public class shoot implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                shooter.setRaw(1.0);
                initialized = true;
            }

            // checks lift's current position
            float[] hsv = new float[3];
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);

            float hue = hsv[0]; // Hue is measured in degrees (0-360)

            if ((hue > 145 && hue < 180)) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                shooter.setRaw(1.0);
                intake.runTransfer();
                intake.runIntake();
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }
    public Action shoot() {
        return new shoot();
    }
    private MecanumDrive drive;
    private VisionManager vision;
    private Intake intake;
    private Shooter shooter;
    private Turret turret;
    public ColorSensor colorSensor;
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
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


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
                parkTrajectory.build(),
                shoot()
        ));
    }
}
