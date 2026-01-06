package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@TeleOp(name = "The Plasma", group = "Teleop")
public class ThePlasma extends LinearOpMode {

    private enum State { IDLE, INTAKING, SHOOTING_DYNAMIC, SHOOTING_RAW }
    private State currentState = State.IDLE;

    @Override
    public void runOpMode() {
        // Hardware Init
        Servo latch = hardwareMap.get(Servo.class, "latchServo");
        Intake intake = new Intake(); intake.init(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, telemetry);
        Mecanum mecanum = new Mecanum(); mecanum.Init(hardwareMap);
        VisionManager vision = new VisionManager(hardwareMap, hardwareMap.get(WebcamName.class, "Webcam 1"), new Size(640, 480));

        waitForStart();

        while (opModeIsActive()) {
            // Drive Logic
            double speed = 0.3 + 0.7 * gamepad1.right_trigger;
            mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, speed);

            // Vision Target
            AprilTagDetection target = vision.getTargetDetection(20); // Or loop through IDs

            // --- State Selection ---
            if (gamepad2.b) currentState = State.SHOOTING_DYNAMIC;
            else if (gamepad2.a) currentState = State.SHOOTING_RAW;
            else if (gamepad1.right_bumper) currentState = State.INTAKING;
            else currentState = State.IDLE;

            // --- State Execution ---
            switch (currentState) {
                case SHOOTING_DYNAMIC:
                    shooter.setMode(Shooter.Mode.DYNAMIC);
                    if (shooter.isAtTargetVelocity()) {
                        latch.setPosition(0.3);
                        intake.runIntake();
                    } else {
                        latch.setPosition(0);
                        intake.stopIntake();
                    }
                    break;

                case SHOOTING_RAW:
                    shooter.setRaw(1.0); // Full speed
                    if (shooter.isAtTargetVelocity()) {
                        latch.setPosition(0.3);
                        intake.runIntake();
                    }
                    break;

                case INTAKING:
                    shooter.setRaw(0);
                    intake.runIntake();
                    latch.setPosition(0);
                    break;

                case IDLE:
                    shooter.setIdle(); // Keep motors warm
                    intake.stopIntake();
                    latch.setPosition(0);
                    break;
            }

            shooter.periodic(target);
            telemetry.update();
        }
    }
}