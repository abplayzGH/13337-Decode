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
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.Math;

@Config
@Autonomous(name = "ihateroadrunner", group = "Auto")
public class ihateroadrunner extends LinearOpMode {

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

    @Override
    public void runOpMode() {

        Robot robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        AutoActions autoActions = new AutoActions(robot);

        /* ---------------- INIT ---------------- */
        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, startPose);


        /* ---------------- TRAJECTORIES ---------------- */


        TrajectoryActionBuilder tab1 = drive.actionBuilder(startPose)
// Preload score
//                .stopAndAdd(autoActions.shooterIdle())
//                .afterTime(0, autoActions.shooterIdle())
//                .setTangent(GOAL_HEADING)
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING);
//                .stopAndAdd(autoActions.fire())
//                .afterTime(2, autoActions.stopShooter())

        TrajectoryActionBuilder tab2 = drive.actionBuilder(startPose)

                //Go to Middle Balls
//                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(SPIKE_2, Math.toRadians(90)), Math.toRadians(0));

        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                //Intake Middle Balls
//                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(SPIKE_2_FINAL, Math.toRadians(90));
//                .stopAndAdd(autoActions.stopIntake())

        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
//              // Return to goal smoothly
//                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING);
//                .stopAndAdd(autoActions.fire())
//                .afterTime(2, autoActions.stopShooter())
        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
//              // Open gate
//                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90));
//                .afterTime(.1, autoActions.stopIntake())

        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()
                //Return to goal
//                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING);
//                .stopAndAdd(autoActions.fire())
//                .afterTime(2, autoActions.stopShooter())

        TrajectoryActionBuilder tab7 = tab6.endTrajectory().fresh()
                // Open gate
//                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90));
//                .afterTime(.1, autoActions.stopIntake())


        TrajectoryActionBuilder tab8 = tab7.endTrajectory().fresh()
                //Return to goal
//                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING);
//                .stopAndAdd(autoActions.fire())
//                .afterTime(2, autoActions.stopShooter());

        /* ---------------- RUN AUTO ---------------- */
        Action act1 = tab1.build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        act1
                )
        );
    }
}