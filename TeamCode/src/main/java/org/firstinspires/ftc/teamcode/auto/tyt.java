package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;

import android.util.Size;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
@Autonomous(name = "Universal Auto", group = "Auto")
@SuppressWarnings("unused") // Registered via annotation; suppress static 'never used' warning
public class tyt extends LinearOpMode {

    // initialize singleton reference early so static analyzers see it as assigned
    private Robot robot = Robot.get();
    private Shooter shooter;
    private Intake intake;
    private Servo latch;

    // Default to RED alliance
//    private boolean isRed = true;

    /* --- ACTIONS --- */
    public Action spinUpAction() {
        return new Action() {
            private double startTime = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();
                shooter.setMode(Shooter.Mode.RAW);
                shooter.setRaw(1.0);
                shooter.periodic(null);

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
            return false;
        };
    }

    public Action stopShooterAction() {
        return packet -> {
            robot.reset();
            latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

    @Override
    public void runOpMode() {
        // --- ALLIANCE SELECTION LOOP ---
        telemetry.addLine("Press (B) for RED, (X) for BLUE");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Selected Alliance", Robot.alliance == Robot.Alliance.RED ? "RED" : "BLUE");
            telemetry.addData("Status", "Waiting for Start...");
            telemetry.update();
        }

        if (isStopRequested()) return;

        // --- INITIALIZATION ---
        // Initialize Robot singleton and set alliance so Robot.Init can pick correct constants
        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        latch = hardwareMap.get(Servo.class, "latchServo");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        Turret turret = new Turret();
        turret.init(hardwareMap, "turret_motor");

        Mecanum mecanum = new Mecanum();
        mecanum.Init(hardwareMap);

        // Vision: construct and start stream immediately (no local variable needed)
        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        try {
            new VisionManager(hardwareMap, cam, new Size(640, 480)).startDashboardStream(15);
        } catch (Exception ignored) {}

        // --- POSE SETUP ---
        // All poses written as RED
        Pose2d startPose = new Pose2d(60, 12, toRadians(180));
        Pose2d goalPose = new Pose2d(-10, 10, toRadians(135));
        Vector2d parkVec = new Vector2d(-40, 10);

        // If BLUE alliance, mirror start pose for odometry/drive initialization
        Pose2d finalStartPose = Robot.alliance == Robot.Alliance.RED ? startPose :
                new Pose2d(startPose.position.x, -startPose.position.y, -startPose.heading.toDouble());

        // --- DRIVE INIT ---
        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStartPose);

        // --- TRAJECTORIES ---
        TrajectoryActionBuilder driveToShootPos =
                drive.actionBuilder(startPose)
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
                        spinUpAction(),
                        fireAction(),
                        new SleepAction(1.5),
                        stopShooterAction(),
                        park
                )
        );
    }
}
