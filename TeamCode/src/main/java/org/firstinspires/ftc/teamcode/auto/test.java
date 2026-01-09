package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
@Autonomous(name = "Test", group = "Autonomous")
public class test extends LinearOpMode {

    public class ShooterSub {
        private Shooter shooter;
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
                    shooter.setRaw(1.0);

                    // 2. CRITICAL: Actually tell the hardware to move
                    // Your Shooter class needs this to run the switch/case logic!
                    shooter.periodic(null);

                    packet.put("Velo", shooter.isAtTargetVelocity());

                    boolean atSpeed = shooter.isAtTargetVelocity();
                    boolean timeout = (System.currentTimeMillis() - startTime) > 2500;

                    return !(atSpeed || timeout);
                }
            };
        }

        public Action fire() {
            return packet -> {
                telemetry.addLine("FIRE");
                latch.setPosition(0.1);
                intake.runTransfer();
                intake.runIntake();
                return false;
            };
        }

        public Action stop() {
            return packet -> {
                shooter.setRaw(0);
                latch.setPosition(0);
                intake.stopIntake();
                return false;
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

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // ... Trajectories ...

        Action driveToShootPos = drive.actionBuilder(startPose)
                .lineToY(36)
                .strafeTo(new Vector2d(36, 36))
                .build();

        Action park = drive.actionBuilder(new Pose2d(36, 36, Math.toRadians(-90)))
                .splineTo(new Vector2d(60, 60), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        driveToShootPos,
                        shooter.spinUp(),      // Spins flywheels until fast
                        shooter.fire(),        // Opens latch + starts intake/transfer
                        new SleepAction(1.5),  // WAIT: Give it 1.5s to actually shoot the ball
                        shooter.stop()         // Everything turns off
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