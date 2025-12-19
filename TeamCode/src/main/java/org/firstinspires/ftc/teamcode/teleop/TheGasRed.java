package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "The Gas Red", group = "Teleop")
public class TheGasRed extends LinearOpMode {

    double speedMult = 0.3;
//    private ballShooter shooter = new ballShooter();
    private Intake intake = new Intake();
    private Shooter shooter;
    private VisionManager visionManager = null;
    private Turret turretSubsystem = null;

    // --- CONFIGURATION ---
    private static final int TARGET_TAG_ID = 24;
    private static final String CAMERA_NAME = "Webcam 1";
    private static final String MOTOR_NAME = "turret_motor";

    @Override
    public void runOpMode() {
        // Initialize hardware
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        Servo latch = hardwareMap.get(Servo.class, "latchServo");

        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        turretSubsystem = new Turret();
        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionManager = new VisionManager(hardwareMap, cam , new Size(640, 480));
        turretSubsystem.init(hardwareMap, MOTOR_NAME);


//        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);


        // Wait for the start button to be pressed
        waitForStart();
        shooter.setMode(Shooter.Mode.RAW);
        latch.setPosition(0);

        while (opModeIsActive()) {
            // Get joystick inputs
            // X = gamepad1 = wheel movement
            double leftX = gamepad1.left_stick_x; // Forward/Backward
            double leftY = gamepad1.left_stick_y;  // Left/Right
            double rightX = -gamepad1.right_stick_x; // Rotate


            //Calculate motor power
            double frontLeftPower = leftY - leftX + rightX;
            double rearLeftPower = leftY + leftX + rightX;
            double frontRightPower = leftY + leftX - rightX;
            double rearRightPower = leftY - leftX - rightX;


            // Normalize the power values if any exceed 1.0
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(rearLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                rearLeftPower /= maxPower;
                frontRightPower /= maxPower;
                rearRightPower /= maxPower;
            }

            speedMult = 0.3 + (0.7*gamepad1.right_trigger);


            // Set wheel power
            leftFront.setPower(frontLeftPower * speedMult);
            leftBack.setPower(rearLeftPower * speedMult);
            rightFront.setPower(frontRightPower * speedMult);
            rightBack.setPower(rearRightPower * speedMult);


            // --- CORRECTED SHOOTER LOGIC ---
            // Determine if a shot is being requested by the operator
            boolean shotRequested = gamepad2.b;

            // The 'startLauncher()' method is redundant if 'updateState' is handled correctly.
            // We can let the state machine handle the SPIN_UP transition.


            // Always update the shooter's state machine.
            // Pass 'true' if the B button is pressed, which will trigger the launch sequence.
            // --- END OF CORRECTION ---

//
//            while (gamepad2.left_bumper) {
//                intake.runIntake();4

            if (gamepad2.a) {
                // Fixed Velocity: Use a predetermined "long-shot" velocity
                shooter.setRaw(1); // Example high velocity
                if (shooter.getVelocity() > 500) {
                    latch.setPosition(0.5);
//                    intake.servoTest();
//                    intake.runIntake();
//                    latch.setPosition(1);

                }
            }else {
                shooter.setRaw(0);
                intake.stopIntake();
                latch.setPosition(0);
            }

            if (gamepad2.left_bumper){
                intake.runIntake();
            } else {
                intake.stopIntake();
            }

            if (gamepad2.right_bumper){
                intake.runIntake();
            } else {
                intake.stopIntake();
            }

//            if (gamepad2.right_bumper){
//                latch.setPosition(.5);
//            } else {
//                latch.setPosition(0);
//            }


//            } else if (gamepad2.b) {
//                // Fixed Velocity: Use a predetermined "short-shot" velocity
//                shooter.setVelocity(-1000); // Example low velocity
//            } else if (gamepad2.x) {
//                // Dynamic Velocity: Use AprilTag vision to determine velocity
//                shooter.mode = Shooter.Mode.DYNAMIC;
//            } else if (gamepad2.y) {
//                // Idle Mode: Keep the motors spinning slowly
//                shooter.setIdle();
//            } else if (gamepad2.dpad_up) {
//                // RAW Mode: Manual control with gamepad stick
//                shooter.setRaw(-gamepad2.right_stick_y);
//            }


//            shooter.setRaw(1); // Example high velocity


            AprilTagDetection detection = visionManager.getTargetDetection(TARGET_TAG_ID);
//
            // B. Pass the data to the TurretSubsystem for processing
            double motorPower = turretSubsystem.updateTurretTracking(detection, getRuntime());

            // C. Telemetry (displaying status)
            if (detection != null) {
                telemetry.addData("Status", "Tracking Tag %d", TARGET_TAG_ID);
                telemetry.addData("Bearing Error", "%.2f", 0.0 - detection.ftcPose.bearing); // Target - Current
            } else {
                telemetry.addData("Status", "Searching for Tag %d...", TARGET_TAG_ID);
            }
            telemetry.addData("Motor Power", "%.2f", motorPower);

            shooter.periodic(detection);

            // Telemetry for debugging (optional)
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RR Power", rearRightPower);
            telemetry.addData("Flywheel Velocity", shooter.getVelocity());

            //Update State
            telemetry.update();
        }
    }

}

