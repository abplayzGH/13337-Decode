package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

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

    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, startPose);

//        Turret turret = new Turret();
//        turret.init(hardwareMap);

        Robot robot = Robot.get();
        robot.Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        /* ---------------- TRAJECTORIES ---------------- */

        TrajectoryActionBuilder tab1 = drive.actionBuilder(startPose)

                // Preload score
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(GOAL_HEADING)
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .afterTime(2, autoActions.stopShooter())

                //Go to Middle Balls
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(SPIKE_2, Math.toRadians(90)), Math.toRadians(0))

                //Intake Middle Balls
                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(SPIKE_2_FINAL, Math.toRadians(90))
                .stopAndAdd(autoActions.stopIntake())

//              // Return to goal smoothly
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .afterTime(2, autoActions.stopShooter())
//
//              // Open gate
                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))
                .afterTime(.1, autoActions.stopIntake())

                //Return to goal
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .afterTime(2, autoActions.stopShooter())

                // Open gate
                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))
                .afterTime(.1, autoActions.stopIntake())

                //Return to goal
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .afterTime(2, autoActions.stopShooter());

//                // Open go to spike 3
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(SPIKE_3, Math.toRadians(90)), Math.toRadians(90))
//
//                // Intake spike 3
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(SPIKE_3_FINAL, Math.toRadians(90))
//
//
//                //Return to goal
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING);


        waitForStart();


        if (isStopRequested()) return;
        /* ---------------- RUN AUTO ---------------- */
        Actions.runBlocking(
                new ParallelAction(
                        tab1.build(), // The driving path
                        autoActions.updateLL(),
                        autoActions.turretTrack(),
                        autoActions.updateShooter()
                )
        );
    }
}