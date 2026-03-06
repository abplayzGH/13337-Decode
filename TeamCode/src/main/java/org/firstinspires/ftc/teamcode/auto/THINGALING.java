package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;

import java.lang.Math;

@Config
@Autonomous(name = "?Why", group = "Auto")
public class THINGALING extends LinearOpMode {
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
    final Pose2d startPose = new Pose2d(-57, 43, Math.toRadians(125));
    final Vector2d Gate = new Vector2d(10, 55);


    /* ---------------- ACTIONS ---------------- */

    @Override
    public void runOpMode() {

        /* ---------------- INIT ---------------- */
        Robot robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        AutoActions autoActions = new AutoActions(robot);

        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, startPose);

        /* ---------------- TRAJECTORIES ---------------- */
        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder tab = drive.actionBuilder(startPose)
                // Preload score
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .stopAndAdd(autoActions.stopShooter())

                //Go to Middle Balls
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(SPIKE_2, Math.toRadians(90)), Math.toRadians(0))

                //Intake Middle Balls
                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(SPIKE_2_FINAL, Math.toRadians(90))
                .stopAndAdd(autoActions.stopIntake())
//
////              // Return to goal smoothly
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(220))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .stopAndAdd(autoActions.stopShooter())
////
////              // Open gate
                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))
                .waitSeconds(2)
                .afterTime(1, autoActions.stopIntake())
//
//                //Return to goal
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .stopAndAdd(autoActions.stopShooter())
//
//                // Open gate
                .stopAndAdd(autoActions.intake())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(Gate, Math.toRadians(115)), Math.toRadians(90))
                .waitSeconds(2)
                .afterTime(1, autoActions.stopIntake())
//
//                //Return to goal
                .stopAndAdd(autoActions.shooterIdle())
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .stopAndAdd(autoActions.stopShooter());

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


        /* ---------------- RUN AUTO ---------------- */
// In THINGALING.java, change your runBlocking to:
        Actions.runBlocking(
                new RaceAction( // Ends when the first action (the driving) finishes
                        new SequentialAction(tab.build()),
                        new ParallelAction(
                                autoActions.updateLL(),
                                autoActions.turretTrack(),
                                autoActions.updateShooter(),

                        (packet) -> {
                            // Add any custom telemetry here
                            packet.put("Robot Pose X", drive.localizer.getPose().position.x);
                            packet.put("Robot Pose Y", drive.localizer.getPose().position.y);
                            packet.put("Turret Pos", robot.turret.getPosition());

                            // Return true to keep this "telemetry loop" running
                            return true;
                        }
                        )
                )
        );

    }
}