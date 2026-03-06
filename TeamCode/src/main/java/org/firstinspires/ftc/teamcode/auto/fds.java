package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Assuming you are using the default Pinpoint or ThreeDeadWheel drive class
// Change 'MecanumDrive' to your specific drive class name if different
import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(name = "RR_V1_Template", group = "Autonomous")
public class fds extends LinearOpMode {
    double GOAL_HEADING = Math.toRadians(135);
    final Vector2d GOAL = new Vector2d(-36, 30);
    final Pose2d GOAL_POSE = new Pose2d(GOAL, GOAL_HEADING);
    @Override
    public void runOpMode() {
        // 1. Initialize Robot FIRST (Now fixed with the Mode check above)
        Robot robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);
        AutoActions autoActions = new AutoActions(robot);

        // 2. Set Start Pose and Drive
        Pose2d initialPose = new Pose2d(-49, 49, Math.toRadians(125));
        MecanumDriveRR drive = new MecanumDriveRR(hardwareMap, initialPose);

        // 3. Build the trajectory BEFORE waitForStart
        Action tray1 = drive.actionBuilder(initialPose)
                .stopAndAdd(autoActions.shooterIdle()) // This now works!
                .strafeTo(new Vector2d(0,0))
                .build();

        telemetry.addLine("Robot Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 4. Run the action
        Actions.runBlocking(tray1);
    }
}