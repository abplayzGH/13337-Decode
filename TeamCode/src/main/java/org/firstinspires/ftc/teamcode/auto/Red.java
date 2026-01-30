package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Math.toRadians;
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

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@Autonomous(name = "Red", group = "Auto")
public class Red extends LinearOpMode {

    private Robot robot;

    /* --- ROADRUNNER ACTIONS FOR ROBOT CLASS --- */

    public Action spinUpAction() {
        return new Action() {
            private double startTime = -1;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) startTime = System.currentTimeMillis();

                // Use the robot class to set motor states
                robot.shooter.setMode(Shooter.Mode.RAW);
                robot.shooter.setRaw(1.0);
                robot.shooter.periodic(null); // Run the PID/FF logic

                boolean atSpeed = robot.shooter.getVelocity() >= 350;
                boolean timeout = (System.currentTimeMillis() - startTime) > 2500;

                return !(atSpeed || timeout);
            }
        };
    }

    public Action fireAction() {
        return packet -> {
            robot.latch.setPosition(Robot.LATCH_OPEN);
            robot.intake.runTransfer();
            robot.intake.runIntake();
            return false; // Action is complete immediately
        };
    }

    public Action stopShooterAction() {
        return packet -> {
            robot.reset(); // Uses your built-in reset logic
            robot.latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

    @Override
    public void runOpMode() {
        // 1. Initialize the Robot Class
        // Ensure you set the alliance BEFORE Init so TARGET_TAG is correct
        Robot.alliance = Robot.Alliance.RED;
        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);

        // 2. Setup Roadrunner Drive
        Pose2d startPose = new Pose2d(60, 12, toRadians(180));
        Vector2d goalVec = new Vector2d(-10, 10);
        Pose2d goalPose = new Pose2d(goalVec, Math.toRadians(135));
        Vector2d parkVec = new Vector2d(-40,10);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

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