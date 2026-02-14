package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import android.util.Size;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.vision.VisionManager;

@Config
@Autonomous(name = "Start at Goal", group = "Auto")
public class Goal extends LinearOpMode {

    private Robot robot;
    private Shooter shooter;
    private Intake intake;
    private Servo latch;
    private boolean isRed = true;
    private int targetTag = 24;

    /* --- ROADRUNNER ACTIONS FOR ROBOT CLASS --- */

    public Action spinUpAction() {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();

                // Use the robot class to set motor states
                shooter.setMode(Shooter.Mode.RAW);
                shooter.setRaw(1.0);
                shooter.periodic(null); // Run the PID/FF logic

                boolean atSpeed = shooter.getVelocity() >= 350;
                boolean timeout = (System.currentTimeMillis() - startTime) > 2500;

                return !(atSpeed || timeout);
            }
        };
    }

    public Action fireAction() {
        return packet -> {
            latch.setPosition(Robot.LATCH_OPEN);
            intake.runTransfer();
            intake.runIntake();
            return false; // Action is complete immediately
        };
    }

    public Action stopShooterAction() {
        return packet -> {
            robot.reset(); // Uses your built-in reset logic
            latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

    @Override
    public void runOpMode() {
        // 1. Initialize the Robot Class
        // Ensure you set the alliance BEFORE Init so TARGET_TAG is correct

        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);
//        Robot.alliance = Robot.Alliance.RED;


        latch = hardwareMap.get(Servo.class, "latchServo");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        /* ---------------- SUBSYSTEMS ---------------- */
        Intake intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        Turret turret = new Turret();
        turret.init(hardwareMap, "turret_motor");

//        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480)); //TODO Tune webcam

        Mecanum mecanum = new Mecanum();
        mecanum.Init(hardwareMap);

        PoseMap poseMap = Robot.alliance == Robot.Alliance.RED ? (pose -> pose) :
                (pose -> new Pose2dDual<>(
                        pose.position.x,
                        pose.position.y.unaryMinus(),
                        pose.heading.inverse()));


        // 2. Setup Roadrunner Drive
        Pose2d startPose = new Pose2d(60, 12, toRadians(180));
        Vector2d goalVec = new Vector2d(-10, 10);
        Pose2d goalPose = new Pose2d(goalVec, Math.toRadians(135));
        Vector2d parkVec = new Vector2d(-40,10);

        Pose2d finalStartPose = isRed ? startPose :
                new Pose2d(startPose.position.x, -startPose.position.y, -startPose.heading.toDouble());

        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStartPose);




        // 3. Build Trajectories
        TrajectoryActionBuilder driveToShootPos = drive.actionBuilder(startPose)
                .splineToLinearHeading(goalPose, 1);

        Action park = drive.actionBuilder(goalPose)
                .strafeToLinearHeading(parkVec, Math.toRadians(180))
                .build();

        waitForStart();
        if (isStopRequested()) return;


        // 4. Run the Sequence
        Actions.runBlocking(
                new SequentialAction(
                        driveToShootPos.build(),
                        spinUpAction(),       // Wrapped Robot call
                        fireAction(),         // Wrapped Robot call
                        new SleepAction(1.5), // Wait for the shot to clear
                        stopShooterAction(),  // Wrapped Robot call
                        park
                )
        );
    }
}