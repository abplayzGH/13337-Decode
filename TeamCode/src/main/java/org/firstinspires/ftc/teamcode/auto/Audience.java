package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
import com.acmerobotics.roadrunner.Pose2dDual;

@Config
@Autonomous(name = "Audience", group = "Auto")
public class Audience extends LinearOpMode {

    private Robot robot;
    private Shooter shooter;
    private Intake intake;
    private Servo latch;

    @Override
    public void runOpMode() {
        // 1. Initialize the Robot Class
        // Ensure you set the alliance BEFORE Init so TARGET_TAG is correct

        Robot robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        AutoActions autoActions = new AutoActions(robot);

//        Robot.alliance = Robot.Alliance.RED;

        /* ---------------- SUBSYSTEMS ---------------- */
        double GOAL_HEADING = Math.toRadians(135);
        final Vector2d GOAL = new Vector2d(-32, 34);
        final Pose2d GOAL_POSE = new Pose2d(GOAL, GOAL_HEADING);
        final Vector2d PARK = new Vector2d(37, -33);
        final Vector2d SPIKE_3 = new Vector2d(-12, 20);
        final Vector2d SPIKE_2 = new Vector2d(12, 22);
        final Vector2d SPIKE_1 = new Vector2d(36, 25);
        final Vector2d SPIKE_3_FINAL = new Vector2d(-12, 50);
        final Vector2d SPIKE_2_FINAL = new Vector2d(12, 54);
        final Vector2d SPIKE_1_FINAL = new Vector2d(36, 50);
//        final Pose2d START_POSE = new Pose2d(60, -12, Math.toRadians(180));
        final Pose2d startPose = new Pose2d(-57, 43, Math.toRadians(125));
        final Pose2d START_POSE2 = new Pose2d(55, 10, Math.toRadians(180));

//        final Pose2d Gate = new Pose2d(-2, 45, Math.toRadians(90));
        final Vector2d Gate = new Vector2d(10.5, 55.5);
        double GOAL_BACk_HEADING = Math.toRadians(156);
        final Vector2d GOAL_BACK = new Vector2d(54, 12);
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

        TrajectoryActionBuilder shoot = drive.actionBuilder(startPose, poseMap)
                .splineToLinearHeading(GOAL_BACK_POSE, Math.toRadians(180));

        TrajectoryActionBuilder end3balls = shoot.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(SPIKE_1_FINAL, Math.toRadians(90)), Math.toRadians(90));

        TrajectoryActionBuilder intake1 = shoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(58, 60), Math.toRadians(90));

        TrajectoryActionBuilder intake2 = shoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, 60), Math.toRadians(90));

        TrajectoryActionBuilder intake3 = shoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(42, 60), Math.toRadians(90));

        TrajectoryActionBuilder end = shoot.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, 25), Math.toRadians(90));



        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new RaceAction( // Ends when the first action (the driving) finishes
                        new SequentialAction(
                                new ParallelAction(
                                        shoot.build(), // Drive to shooting position and spinup
                                        autoActions.shooterIdle()
                                ),
                                autoActions.fire(),


                                new ParallelAction(
                                        end3balls.build(),
                                        autoActions.intake()
                                ),
                                autoActions.stopIntake(),

                                new ParallelAction(
                                        shoot.build(),
                                        autoActions.shooterIdle()
                                ),
                                autoActions.fire(),

                                new ParallelAction(
                                        intake1.build(),
                                        autoActions.intake()
                                ),
                                autoActions.stopIntake(),

                                new ParallelAction(
                                        shoot.build(),
                                        autoActions.shooterIdle()
                                ),
                                autoActions.fire(),

                                new ParallelAction(
                                        intake2.build(),
                                        autoActions.intake()
                                ),
                                autoActions.stopIntake(),

                                new ParallelAction(
                                        shoot.build(),
                                        autoActions.shooterIdle()
                                ),
                                autoActions.fire(),

                                new ParallelAction(
                                        intake3.build(),
                                        autoActions.intake()
                                ),
                                autoActions.stopIntake(),

                                new ParallelAction(
                                        shoot.build(),
                                        autoActions.shooterIdle()
                                ),
                                autoActions.fire(),

                                end.build()


                        ),
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