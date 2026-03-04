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
import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.Math;

@Config
@Autonomous(name = "Why", group = "Auto")
public class why extends LinearOpMode {

    AutoActions autoActions = new AutoActions();

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
    final Pose2d startPose = new Pose2d(-49, 49, Math.toRadians(125));
    final Vector2d Gate = new Vector2d(10, 55);


    /* ---------------- ACTIONS ---------------- */

    public class TurretTrackAction implements Action {
        private Turret turret;
        private MecanumDriveRR drive;
        private Vector2d target;

        public TurretTrackAction(Turret t, MecanumDriveRR d, Vector2d goal) {
            this.turret = t;
            this.drive = d;
            this.target = goal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // This runs every loop cycle
            turret.updateFieldCentric(drive.localizer.getPose(), target);

            // Return true to keep the action running indefinitely
            // (until the ParallelAction it's in finishes)
            return true;
        }
    }
    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, startPose);

        Turret turret = new Turret();
        turret.init(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        /* ---------------- TRAJECTORIES ---------------- */

        TrajectoryActionBuilder toFirstGoal = drive.actionBuilder(startPose)

                // Preload score
                .setTangent(GOAL_HEADING)
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)

                //Go to Middle Balls
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(SPIKE_2, Math.toRadians(90)), Math.toRadians(0))

                //Intake Middle Balls
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(SPIKE_2_FINAL, Math.toRadians(90))

//                        // Return to goal smoothly
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
//
//                        // Open gate
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))

                //Return to goal
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)

                // Open gate
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))

                //Return to goal
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)

                // Open go to spike 3
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(SPIKE_3, Math.toRadians(90)), Math.toRadians(90))

                // Intake spike 3
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(SPIKE_3_FINAL, Math.toRadians(90))


                //Return to goal
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING);



        /* ---------------- RUN AUTO ---------------- */
        Actions.runBlocking(
                new ParallelAction(
                        toFirstGoal.build(), // The driving path
                        new TurretTrackAction(turret, drive, GOAL) // The tracking logic
                )
        );
    }
}