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
@Autonomous(name = "Universal Auto", group = "Auto")
public class tyt extends LinearOpMode {

    private Robot robot;
    private Shooter shooter;
    private Intake intake;
    private Servo latch;

    // Default to RED alliance
    private boolean isRed = true;

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
            if (gamepad1.b) isRed = true;
            if (gamepad1.x) isRed = false;

            telemetry.addData("Selected Alliance", isRed ? "RED" : "BLUE");
            telemetry.addData("Status", "Waiting for Start...");
            telemetry.update();
        }

        // --- INITIALIZATION (Happens immediately after start is pressed) ---

        // 1. Set Robot Global Alliance
        Robot.alliance = isRed ? Robot.Alliance.RED : Robot.Alliance.BLUE;
        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        // 2. Hardware Init
        latch = hardwareMap.get(Servo.class, "latchServo");
        hardwareMap.get(ColorSensor.class, "colorSensor");

        intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        Turret turret = new Turret();
        turret.init(hardwareMap, "turret_motor");

        Mecanum mecanum = new Mecanum();
        mecanum.Init(hardwareMap);

        // Vision (Caution: Ensure VisionManager handles the alliance change internally if needed)
        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480));

        // 3. Define Pose Map
        // If RED: Do nothing (Identity). If BLUE: Mirror Y and Heading.
        PoseMap poseMap = isRed ?
                (pose -> pose) :
                (pose -> new Pose2dDual<>(
                        pose.position.x,
                        pose.position.y.unaryMinus(),
                        pose.heading.inverse()));

        // 4. Setup Drive with Start Pose
        // Note: Write your coordinates as if you are always RED.
        Pose2d startPose = new Pose2d(60, 12, toRadians(180));
        Vector2d goalVec = new Vector2d(-10, 10);
        Pose2d goalPose = new Pose2d(goalVec, Math.toRadians(135));
        Vector2d parkVec = new Vector2d(-40, 10);

        // Initialize Drive with the correct starting pose
        // IMPORTANT: If Blue, we must manually flip the startPose for the LOCALIZER initialization
        Pose2d finalStartPose = isRed ? startPose :
                new Pose2d(startPose.position.x, -startPose.position.y, -startPose.heading.toDouble());

        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStartPose);

        // 5. Build Trajectories using the poseMap
        // Pass 'poseMap' to the actionBuilder. It will flip the targets automatically for Blue.
        TrajectoryActionBuilder driveToShootPos = drive.actionBuilder(startPose)
                .splineToLinearHeading(goalPose, 1);

        Action park = drive.actionBuilder(goalPose)
                .strafeToLinearHeading(parkVec, Math.toRadians(180))
                .build();

        // 6. Run the Sequence
        if (isStopRequested()) return;

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