package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import com.acmerobotics.roadrunner.Pose2dDual;

@Config
@Autonomous(name = "test", group = "Auto")
public class Depot extends LinearOpMode {

    private Robot robot;
    private Shooter shooter;
    private Intake intake;
    private Servo latch;

    @Override
    public void runOpMode() {
        // 1. Initialize the Robot Class
        // Ensure you set the alliance BEFORE Init so TARGET_TAG is correct

        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);
        AutoActions autoActions = new AutoActions();

//        Robot.alliance = Robot.Alliance.RED;

        /* ---------------- SUBSYSTEMS ---------------- */
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

        PoseMap poseMap = Robot.alliance == Robot.Alliance.RED
                ? (pose -> pose)
                : (pose -> new Pose2dDual<>(
                pose.position.x,
                pose.position.y.unaryMinus(),
                pose.heading.inverse()
        ));

        Pose2d mappedStartPose = Robot.alliance == Robot.Alliance.RED
                ? startPose
                : new Pose2d(
                startPose.position.x,
                -startPose.position.y,
                -startPose.heading.toDouble()
        );

        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, mappedStartPose);


        Action auto = drive.actionBuilder(startPose, poseMap)

                // Preload score
                .afterTime(0.0, autoActions.spinUp())
                .setTangent(GOAL_HEADING)
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .waitSeconds(1.5)
                .stopAndAdd(autoActions.stopShooter())


                //Go to Middle Balls
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(SPIKE_2, Math.toRadians(90)), Math.toRadians(0))


                //Intake Middle Balls
                .afterTime(0.0, autoActions.intake())
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(SPIKE_2_FINAL, Math.toRadians(90))
                .afterTime(1.0, autoActions.spinUp())


                // Return to goal smoothly
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .stopAndAdd(autoActions.fire())
                .waitSeconds(1.5)
                .stopAndAdd(autoActions.stopShooter())

                // Open gate
                .afterTime(1, autoActions.intake())
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
                .splineToLinearHeading(GOAL_POSE, GOAL_HEADING)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(auto);
    }
}