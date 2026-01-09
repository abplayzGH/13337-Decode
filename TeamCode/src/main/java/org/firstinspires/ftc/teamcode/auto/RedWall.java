package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
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

import java.lang.Math;

@Config
@Autonomous(name = "Start at red wall", group = "Autonomous")
public class RedWall extends LinearOpMode {

    /* ---------------- CONSTANTS ---------------- */
    public static double LATCH_OPEN = 0.1;
    public static double LATCH_CLOSED = 0.0;
    public static double SHOOT_POWER = 1.0;

    private static final int[] TARGET_TAGS = {20, 24};

    double GOAL_HEADING = Math.toRadians(135);
    final Vector2d GOAL = new Vector2d(-36, 30);
    final Pose2d GOAL_POSE = new Pose2d(GOAL, GOAL_HEADING);
    final Vector2d PARK = new Vector2d(37, -33);
    final Vector2d SPIKE_3 = new Vector2d(-12, 25);
    final Vector2d SPIKE_2 = new Vector2d(12, 25);
    final Vector2d SPIKE_1 = new Vector2d(36, 25);
    final Vector2d SPIKE_3_FINAL = new Vector2d(-12, 50);
    final Vector2d SPIKE_2_FINAL = new Vector2d(12, 50);
    final Vector2d SPIKE_1_FINAL = new Vector2d(36, 50);
    final Pose2d START_POSE = new Pose2d(60, -12, Math.toRadians(180));


    /* ---------------- ACTIONS ---------------- */

    public class ShootAction implements Action {
        private final Shooter shooter;
        private final Intake intake;
        private final Servo latch;
        private boolean fired = false;

        public ShootAction(Shooter shooter, Intake intake, Servo latch) {
            this.shooter = shooter;
            this.intake = intake;
            this.latch = latch;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!fired) {
                shooter.setRaw(SHOOT_POWER);
                if (shooter.isAtTargetVelocity()) {
                    latch.setPosition(LATCH_OPEN);
                    intake.runTransfer();
                    intake.runIntake();
                    fired = true;
                }
                return true;
            } else {
                latch.setPosition(LATCH_CLOSED);
                intake.stopIntake();
                shooter.setRaw(0);
                return false;
            }
        }
    }

    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        Intake intake = new Intake();
        intake.init(hardwareMap);

        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        Turret turret = new Turret();
        turret.init(hardwareMap, "turret_motor");

        Servo latch = hardwareMap.get(Servo.class, "latchServo");
        latch.setPosition(LATCH_CLOSED);

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480));

        /* ---------------- VISION INIT LOOP ---------------- */
        AprilTagDetection tag = null;
        while (!isStarted() && !isStopRequested()) {
            for (int id : TARGET_TAGS) {
                tag = vision.getTargetDetection(id);
                if (tag != null) break;
            }
            telemetry.addData("Tag", tag != null ? tag.id : "None");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        /* ---------------- TRAJECTORIES ---------------- */

        TrajectoryActionBuilder toFirstGoal = drive.actionBuilder(START_POSE)
                .splineToSplineHeading(GOAL_POSE, Math.toRadians(90));

        TrajectoryActionBuilder parkTrajectory = drive.actionBuilder(START_POSE)
                .strafeTo(PARK);

        TrajectoryActionBuilder toSpike3 = drive.actionBuilder(START_POSE)
                .strafeTo(SPIKE_3);

        TrajectoryActionBuilder driveIntoSpike3 = drive.actionBuilder(START_POSE)
                .strafeTo(SPIKE_3_FINAL);

        TrajectoryActionBuilder toSpike2 = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(SPIKE_2, Math.toRadians(90));

        TrajectoryActionBuilder driveIntoSpike2 = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(SPIKE_2, Math.toRadians(90));

        TrajectoryActionBuilder toSpike1 = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(SPIKE_2, Math.toRadians(90));

        TrajectoryActionBuilder driveIntoSpike1 = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(SPIKE_2, Math.toRadians(90));


        /* ---------------- RUN AUTO ---------------- */
        Actions.runBlocking(
                new SequentialAction(
                        toFirstGoal.build(),
                        new ShootAction(shooter, intake, latch),

                        toSpike3.build(),
                        driveIntoSpike3.build(),
                        toFirstGoal.build(),
                        new ShootAction(shooter, intake, latch),

                        toSpike2.build(),
                        driveIntoSpike2.build(),
                        toFirstGoal.build(),
                        new ShootAction(shooter, intake, latch),

                        toSpike1.build(),
                        driveIntoSpike1.build(),
                        toFirstGoal.build(),
                        new ShootAction(shooter, intake, latch),

                        parkTrajectory.build()
                )
        );
    }
}
