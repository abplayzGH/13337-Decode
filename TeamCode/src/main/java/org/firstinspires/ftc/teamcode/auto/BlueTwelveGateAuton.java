package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.IntakeActions;
import org.firstinspires.ftc.teamcode.auto.Turret;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@Autonomous(name = "BLUE - 12 - GATE Auton")
public class BlueTwelveGateAuton extends LinearOpMode {
    public static final double FLYWHEEL_RPM = 2700.0;

    public static final double PITCH_POSITION = 0.67;
    public static final double SHOOT_WAIT_TIME = 1.0;
    public static final double COLLECT_WAIT_TIME = 0.01;
    public static final double GATE_OPEN_TIME = 1.0;
    @Override
    public void runOpMode() {
        final double BLUE_SHOOT_ROTATION = Math.toRadians(-135);
        final double BLUE_COLLECT_ROTATION = Math.toRadians(-90);


        Pose2d initialPose = new Pose2d(-40, -52, BLUE_COLLECT_ROTATION);
        Vector2d shooting = new Vector2d(-14, -14);
        Vector2d collectFirstSet = new Vector2d(-12, -50);
        Vector2d lineUpSecondSet = new Vector2d(12, -22);
        Vector2d collectSecondSet = new Vector2d(12, -57);
        Vector2d lineUpThirdSet = new Vector2d(36, -20);
        Vector2d collectThirdSet = new Vector2d(36, -57);

        Vector2d lineUpGate = new Vector2d(-3, -45);
        Vector2d openGate = new Vector2d(-3, -53);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        IntakeActions intake = new IntakeActions(hardwareMap);
        Turret turret = new Turret(hardwareMap, telemetry);

        // TODO run on init
        //Actions.runBlocking(new SequentialAction(claw.clawClose()));

        Action action = new ParallelAction(
                intake.intakeHold(),
                turret.setFlywheelRPM(FLYWHEEL_RPM),
                drive.actionBuilder(initialPose)
                        // shoot preset balls
                        .waitSeconds(1.5)
                        .setReversed(true)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION + Math.toRadians(3))
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect first spike
                        .turnTo(BLUE_COLLECT_ROTATION)
                        .stopAndAdd(turret.setFlywheelRPM(FLYWHEEL_RPM))
                        .afterTime(0, intake.intakeHold())
                        .strafeTo(collectFirstSet)
                        .waitSeconds(COLLECT_WAIT_TIME)

                        // open gate and shoot
                        .strafeTo(lineUpGate)
                        .strafeTo(openGate)
                        .waitSeconds(GATE_OPEN_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect second spike and shoot
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(lineUpSecondSet, BLUE_COLLECT_ROTATION), BLUE_COLLECT_ROTATION)
                        .afterTime(0, intake.intakeHold())
                        .strafeTo(collectSecondSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(shooting, BLUE_SHOOT_ROTATION), -BLUE_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // collect third spike and shoot
                        .strafeToSplineHeading(lineUpThirdSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, intake.intakeHold())
                        .waitSeconds(0.3)
                        .strafeTo(collectThirdSet)
                        .waitSeconds(COLLECT_WAIT_TIME)
                        .strafeToSplineHeading(shooting, BLUE_SHOOT_ROTATION)
                        .afterTime(0, intake.intakeShoot())
                        .waitSeconds(SHOOT_WAIT_TIME)

                        // reset
                        .strafeToSplineHeading(collectFirstSet, BLUE_COLLECT_ROTATION)
                        .afterTime(0, intake.intakeOff())
                        .afterTime(0, turret.setFlywheelRPM(0))

                        .build()
        );

        waitForStart();
        telemetry.update();

        if (isStopRequested()) return;
        Actions.runBlocking(action);
    }
}