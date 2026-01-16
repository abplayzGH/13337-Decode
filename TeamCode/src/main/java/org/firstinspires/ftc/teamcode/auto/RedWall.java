package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
@Autonomous(name = "Start at red wall", group = "Auto")
public class RedWall extends LinearOpMode {

    /* ---------------- CONSTANTS ---------------- */
    public static double LATCH_OPEN = 0.1;
    public static double LATCH_CLOSED = 0.0;
    public static double SHOOT_POWER = 1.0;

    private static final int[] TARGET_TAGS = {20, 24};

    double GOAL_HEADING = Math.toRadians(135);
    final Vector2d GOAL = new Vector2d(-25, 25);
    final Pose2d GOAL_POSE = new Pose2d(GOAL, GOAL_HEADING);
    final Pose2d PARK = new Pose2d(new Vector2d(37, -33), Math.toRadians(0));
    final Pose2d SPIKE_3 = new Pose2d((new Vector2d(-12, 25)), Math.toRadians(90));
    final Pose2d SPIKE_2 = new Pose2d((new Vector2d(12, 25)), Math.toRadians(90));
    final Pose2d SPIKE_1 = new Pose2d(new Vector2d(36, 25), Math.toRadians(90));
    final Pose2d SPIKE_3_FINAL = new Pose2d(new Vector2d(-12, 50), Math.toRadians(90));
    final Pose2d SPIKE_2_FINAL = new Pose2d(new Vector2d(12, 50), Math.toRadians(90));
    final Pose2d SPIKE_1_FINAL = new Pose2d(new Vector2d(36, 50), Math.toRadians(90));
    final Pose2d START_POSE = new Pose2d(60, 12, Math.toRadians(180)); // Starting away from goal


    /* ---------------- ACTIONS ---------------- */

    public class ShooterSub {
        public Shooter shooter;
        private Servo latch;
        private Intake intake;

        public ShooterSub(HardwareMap hw, Intake sharedIntake) {
            shooter = new Shooter(hw, telemetry);
            latch = hw.get(Servo.class, "latchServo");
            this.intake = sharedIntake;
        }

        public Action spinUp() {
            return new Action() {
                private double startTime = -1;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (startTime < 0) startTime = System.currentTimeMillis();

                    // 1. Set the targets
                    shooter.setMode(Shooter.Mode.FIXED);
                    shooter.setTargetVelocity(700);

                    // 2. CRITICAL: Actually tell the hardware to move
                    // Your Shooter class needs this to run the switch/case logic!
                    shooter.periodic(null);

                    packet.put("Velo", shooter.getVelocity());
                    telemetry.addData("Velocity", shooter.getVelocity());

                    return (shooter.isAtTargetVelocity());
                }
            };
        }
        public Action fire() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    telemetry.addLine("FIRE");
                    latch.setPosition(0.1);
                    intake.runTransfer();
                    intake.runIntake();
                    return false;
                }
            };
        }

        public Action runIntake() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    telemetry.addLine("Intake");
                    latch.setPosition(0);
                    intake.runTransfer();
                    intake.runIntake();
                    return false;
                }
            };
            }

        public Action stop() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    shooter.setRaw(0);
                    latch.setPosition(0);
                    intake.stopIntake();
                    return false;
                }
            };
        }

        public Action idle() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    shooter.setIdle();
                    return false;
                }
            };
        }
    }

    public class TurretSub {
        private Turret turret;
        private VisionManager vision;

        public TurretSub(HardwareMap hw, VisionManager vision) {
            turret = new Turret();
            turret.init(hw, "turret_motor");
            this.vision = vision;
        }

        public Action aimAtTag(int tagId) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    AprilTagDetection target = vision.getTargetDetection(tagId);
                    if (target != null) {
                        turret.updateTurretTracking(target, getRuntime());
                        return Math.abs(target.ftcPose.bearing) > 2.0;
                    }
                    return true;
                }
            };
        }
    }


    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        Intake intake = new Intake();
        intake.init(hardwareMap);

        Servo latch = hardwareMap.get(Servo.class, "latchServo");
        latch.setPosition(LATCH_CLOSED);

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480));

        ShooterSub shooter = new ShooterSub(hardwareMap, intake);
        TurretSub turret = new TurretSub(hardwareMap, vision);

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

        TrajectoryActionBuilder toSpike3 = drive.actionBuilder(GOAL_POSE)
                .splineToLinearHeading(SPIKE_3, 1);

        TrajectoryActionBuilder driveIntoSpike3 = drive.actionBuilder(SPIKE_3)
                .strafeTo(SPIKE_3_FINAL.position);

        TrajectoryActionBuilder toSpike2 = drive.actionBuilder(GOAL_POSE)
                .strafeToLinearHeading(SPIKE_2.position, Math.toRadians(90));

        TrajectoryActionBuilder driveIntoSpike2 = drive.actionBuilder(SPIKE_2)
                .strafeToLinearHeading(SPIKE_2_FINAL.position, Math.toRadians(90));

        TrajectoryActionBuilder toSpike1 = drive.actionBuilder(GOAL_POSE)
                .strafeToLinearHeading(SPIKE_1.position, Math.toRadians(90));

        TrajectoryActionBuilder driveIntoSpike1 = drive.actionBuilder(SPIKE_1)
                .strafeToLinearHeading(SPIKE_1_FINAL.position, Math.toRadians(90));

        TrajectoryActionBuilder parkTrajectory = drive.actionBuilder(GOAL_POSE)
                .strafeTo(PARK.position);


        /* ---------------- RUN AUTO ---------------- */
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                toFirstGoal.build(),
                                shooter.spinUp()
                        ),
                        shooter.fire(),
                        new SleepAction(2),
                        shooter.stop(),

                        toSpike3.build(),

                        new ParallelAction(
                            shooter.runIntake(),
                            driveIntoSpike3.build()
                        ),
                        shooter.stop(),

                        new ParallelAction(
                                toFirstGoal.build(),
                                shooter.spinUp()
                        ),

                        shooter.fire(),
                        new SleepAction(2),
                        shooter.stop(),

                        toSpike2.build(),

                        new ParallelAction(
                            driveIntoSpike2.build(),
                            shooter.runIntake()
                        ),
                        shooter.stop(),

                        new ParallelAction(
                                toFirstGoal.build(),
                                shooter.spinUp()
                        ),
                        shooter.fire(),
                        new SleepAction(2),
                        shooter.stop(),

                        toSpike1.build(),

                        new ParallelAction(
                                driveIntoSpike1.build(),
                                shooter.runIntake()
                        ),

                        new ParallelAction(
                                toFirstGoal.build(),
                                shooter.spinUp()
                        ),

                        shooter.fire(),
                        new SleepAction(2),
                        shooter.stop(),


                        parkTrajectory.build()
                )
        );
    }
}
