package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot; // Using your Robot class
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.Math;

@Config
@Autonomous(name = "Start at red wall", group = "Auto")
public class RedWall extends LinearOpMode {

    private Robot robot;

    /* ---------------- POSITIONS ---------------- */
    // Using Pose2d for consistency
    final Pose2d START_POSE = new Pose2d(60, 12, Math.toRadians(180));
    final Pose2d GOAL_POSE = new Pose2d(-25, 25, Math.toRadians(135));
    final Pose2d SPIKE_3 = new Pose2d(-12, 25, Math.toRadians(90));
    final Pose2d SPIKE_3_FINAL = new Pose2d(-12, 50, Math.toRadians(90));
    // ... (Keep your other pose definitions here)

    /* ---------------- ROADRUNNER ACTIONS ---------------- */

    public Action spinUp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                robot.shooter.setMode(Shooter.Mode.FIXED);
                robot.shooter.setTargetVelocity(700);
                robot.shooter.periodic(null); // Critical: Updates the PID

                packet.put("Shooter Velocity", robot.shooter.getVelocity());

                // Return TRUE to keep running until at velocity
                return !robot.shooter.isAtTargetVelocity();
            }
        };
    }

    public Action fire() {
        return packet -> {
            robot.latch.setPosition(Robot.LATCH_OPEN);
            robot.intake.runTransfer();
            robot.intake.runIntake();
            return false; // Runs once and finishes
        };
    }

    public Action stopAll() {
        return packet -> {
            robot.reset(); // Uses the reset method in your Robot class
            return false;
        };
    }

    public Action intakePixels() {
        return packet -> {
            robot.latch.setPosition(Robot.LATCH_CLOSED);
            robot.intake.runIntake();
            robot.intake.runTransfer();
            return false;
        };
    }

    @Override
    public void runOpMode() {
        // 1. Initialize Robot Class (The "Brain")
        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        // 2. Initialize Drive (Roadrunner)
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        /* ---------------- TRAJECTORIES ---------------- */
        Action toFirstGoal = drive.actionBuilder(START_POSE)
                .splineToSplineHeading(GOAL_POSE, Math.toRadians(90))
                .build();

        Action spike3Sequence = drive.actionBuilder(GOAL_POSE)
                .splineToLinearHeading(SPIKE_3, Math.toRadians(90))
                .strafeTo(SPIKE_3_FINAL.position)
                .build();

        // (Define Spike 2, Spike 1, and Park similarly...)

        waitForStart();
        if (isStopRequested()) return;

        /* ---------------- EXECUTION ---------------- */
        Actions.runBlocking(
                new SequentialAction(
                        // Drive to goal while spinning up flywheels
                        new ParallelAction(toFirstGoal, spinUp()),

                        // Fire
                        fire(),
                        new SleepAction(1.0),
                        stopAll(),

                        // Go get more pixels
                        new ParallelAction(spike3Sequence, intakePixels()),
                        stopAll()

                        // Repeat for other spikes...
                )
        );
    }
}