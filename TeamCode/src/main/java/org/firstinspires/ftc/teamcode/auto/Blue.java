package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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

import android.util.Size;

@Config
@Autonomous(name = "Blue", group = "Auto")
public class Blue extends LinearOpMode {

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
                    shooter.setMode(Shooter.Mode.RAW);
                    shooter.setRaw(1);

                    // 2. CRITICAL: Actually tell the hardware to move
                    // Your Shooter class needs this to run the switch/case logic!
                    shooter.periodic(null);

                    packet.put("Velo", shooter.isAtTargetVelocity());

                    boolean atSpeed = shooter.getVelocity() >= 350;
                    boolean timeout = (System.currentTimeMillis() - startTime) > 2500;

                    return !(atSpeed || timeout);
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
        // 1. Initialize Subsystems FIRST
        Intake intake = new Intake();
        intake.init(hardwareMap); // Fixes the null pointer error

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480));

        ShooterSub shooter = new ShooterSub(hardwareMap, intake);
        TurretSub turret = new TurretSub(hardwareMap, vision);

        Pose2d startPose = new Pose2d(60, -12, toRadians(180));
        Vector2d goalVec = new Vector2d(-10, -10);
        Pose2d goalPose = new Pose2d(goalVec, Math.toRadians(220));
        Vector2d parkVec = new Vector2d(-40,-10);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // ... Trajectories ...

        TrajectoryActionBuilder driveToShootPos = drive.actionBuilder(startPose)
                .splineToLinearHeading(goalPose, 1);

        Action park = drive.actionBuilder(goalPose)
                .strafeToLinearHeading(parkVec, Math.toRadians(180))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        driveToShootPos.build(),
                        shooter.spinUp(),// Spins flywheels until fast
                        shooter.fire(),        // Opens latch + starts intake/transfer
                        new SleepAction(5),  // WAIT: Give it 1.5s to actually shoot the ball
                        shooter.stop(),// Everything turns off
                        park
                )
        );
    }

    private Action sleepAction(long ms) {
        return new Action() {
            private long startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();
                return (System.currentTimeMillis() - startTime) < ms;
            }
        };
    }
}