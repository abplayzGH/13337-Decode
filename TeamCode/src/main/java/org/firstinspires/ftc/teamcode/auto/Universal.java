package org.firstinspires.ftc.teamcode.auto;


import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnActionFactory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;



@Config
@Autonomous(name = "Universal Auto WIP", group = "Auto")
@SuppressWarnings("unused")

public class Universal extends LinearOpMode {
    @Override
    public void runOpMode() {

//        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Selected Alliance", Robot.alliance == Robot.Alliance.RED ? "RED" : "BLUE");
//            telemetry.addData("Status", "Waiting for Start...");
//            telemetry.update();
//        }
//
//        if (isStopRequested()) return;

        // --- INITIALIZATION ---
        // Initialize Robot singleton and set alliance so Robot.Init can pick correct constants
        // initialize singleton reference early so static analyzers see it as assigned
        Robot robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        AutoActions autoActions = new AutoActions(robot);

        // --- POSE SETUP ---
        // All poses written as RED
        Pose2d startPose = new Pose2d(60, 12, toRadians(180));
        Pose2d goalPose = new Pose2d(-10, 10, toRadians(135));
        Vector2d parkVec = new Vector2d(-40, 10);

        // If BLUE alliance, mirror start pose for odometry/drive initialization
        Pose2d finalStartPose = Robot.alliance == Robot.Alliance.RED ? startPose :
                new Pose2d(startPose.position.x, -startPose.position.y, -startPose.heading.toDouble());


        // --- DRIVE INIT ---
        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, finalStartPose);

        // --- TRAJECTORIES ---
        TrajectoryActionBuilder driveToShootPos =
                drive.actionBuilder(finalStartPose)
                        .splineToLinearHeading(goalPose, 1);

        Action park = drive.actionBuilder(goalPose)
                .strafeToLinearHeading(parkVec, Math.toRadians(180))
                .build();

        // Wait for start (safety)
        waitForStart();
        if (isStopRequested()) return;

        // --- RUN SEQUENCE ---
        Actions.runBlocking(
                new SequentialAction(
                        driveToShootPos.build(),
                        autoActions.fire(),
                        new SleepAction(1.5),
                        autoActions.stopShooter(),
                        park
                )
        );
    }
}
