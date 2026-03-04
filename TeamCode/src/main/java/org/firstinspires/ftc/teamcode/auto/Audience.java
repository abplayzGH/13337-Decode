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
public class Audience extends LinearOpMode {

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
        final Pose2d startPose = new Pose2d(55, 10, Math.toRadians(180));
        final Vector2d Gate = new Vector2d(10, 55);
        double GOAL_BACk_HEADING = Math.toRadians(135);
        final Vector2d GOAL_BACK = new Vector2d(-36, 30);
        final Pose2d GOAL_BACK_POSE = new Pose2d(GOAL_BACK, GOAL_BACk_HEADING);

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

                .splineToLinearHeading(new Pose2d(SPIKE_1_FINAL, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(startPose, Math.toRadians(270))

                .splineToLinearHeading(new Pose2d(new Vector2d(55, 55), Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(startPose, Math.toRadians(270))

                .splineToLinearHeading(new Pose2d(new Vector2d(50, 55), Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(startPose, Math.toRadians(270))

                .splineToLinearHeading(new Pose2d(new Vector2d(45, 55), Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(startPose, Math.toRadians(270))


                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(auto);
    }
}