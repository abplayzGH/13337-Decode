package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@Autonomous(name = "test", group = "Auto")
public class test extends LinearOpMode {

    private Robot robot;
    private Shooter shooter;
    private Intake intake;
    private Servo latch;



    @Override
    public void runOpMode() {
        // 1. Initialize the Robot Class
        // Ensure you set the alliance BEFORE Init so TARGET_TAG is correct

//        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);
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
//        final Pose2d START_POSE = new Pose2d(60, -12, Math.toRadians(180));
        final Pose2d startPose = new Pose2d(-49, 49, Math.toRadians(125));

        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, startPose);


        Action thing = drive.actionBuilder(startPose)
                .strafeTo(GOAL_POSE.position)

                .strafeToLinearHeading(SPIKE_3, Math.toRadians(90))

                .strafeTo(SPIKE_3_FINAL)
//
                .strafeToLinearHeading(GOAL_POSE.position, GOAL_POSE.heading)
//
                .strafeTo(new Vector2d(12, 56))
                .lineToX(20)

                .build();

        waitForStart();
        if (isStopRequested()) return;


        // 4. Run the Sequence
        Actions.runBlocking(
                new SequentialAction(
                    thing
                )
        );
    }
}