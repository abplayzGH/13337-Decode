package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "DecodeAuto_RR", group = "Auto")
public class TestAuto1 extends LinearOpMode {
    // Hardware
    private MecanumDrive drive;  // or whatever your drivetrain class is
    private DcMotorEx liftMotor;
    private Servo manipulatorServo;

    // Vision / sensor class (you’ll implement this)
//    private VisionCam vision;

    // Example positions / constants you will tune
    public static double START_X = 12;
    public static double START_Y = 12;
    public static double START_HEADING = 0;  // radians

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        drive = new MecanumDrive(hardwareMap, new Pose2d(START_X, START_Y, START_HEADING));
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        manipulatorServo = hardwareMap.get(Servo.class, "manipServo");

//        vision = new VisionCam(hardwareMap);  // your class to detect artifacts / zones

        // During init, maybe detect which "pattern" is present (left / center / right)
        int zone = 1;
        while (!isStarted() && !isStopRequested()) {
//            zone = vision.getZone();  // returns 0, 1, 2 (for example)
            telemetry.addData("Detected zone", zone);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Build trajectory options based on detected zone
        // These are speculative — you’ll change them once field layout is known.
        Action trajToScoreZone0 = drive.actionBuilder(drive.localizer.getPose())
                .lineToX(30)
                .lineToY(10)
                .build();
//                .splineTo(new Vector2d(30, 10), Math.toRadians(0))  // spline to (30,10) facing "forward"
//                .splineTo(new Vector2d(30, 25), Math.toRadians(90));

        Action trajToScoreZone1 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(25, 15), Math.toRadians(0))
                .splineTo(new Vector2d(35, 20), Math.toRadians(45))
                .build();

        Action trajToScoreZone2 = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(20, 10), Math.toRadians(0))
                .turn(Math.toRadians(-45))
                .splineTo(new Vector2d(15, 20), Math.toRadians(0))
                .build();

        // After scoring, park / move to decode zone or end zone
        Action parkAction = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(10, 5), Math.toRadians(0))
                .build();

        // Mechanism actions
//        Action liftUp = new Action() {
//            boolean init = false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!init) {
//                    liftMotor.setPower(0.8);
//                    init = true;
//                }
//                if (liftMotor.getCurrentPosition() < 1000) {
//                    packet.put("lift pos", liftMotor.getCurrentPosition());
//                    return true;
//                } else {
//                    liftMotor.setPower(0);
//                    return false;
//                }
//            }
//        };
//
//        Action manipOpen = new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                manipulatorServo.setPosition(1.0);
//                return false;
//            }
//        };
//
//        Action manipClose = new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                manipulatorServo.setPosition(0.0);
//                return false;
//            }
//        };

        // Choose the correct trajectory based on vision
        Action chosenTrajectory;
        switch (zone) {
            case 0: chosenTrajectory = trajToScoreZone0; break;
            case 1: chosenTrajectory = trajToScoreZone1; break;
            case 2: chosenTrajectory = trajToScoreZone2; break;
            default: chosenTrajectory = trajToScoreZone1; break;  // fallback
        }

        // Combine into a sequence:
        SequentialAction fullAuto = new SequentialAction(
                chosenTrajectory,
                parkAction
        );

        Actions.runBlocking(fullAuto);
    }
}
