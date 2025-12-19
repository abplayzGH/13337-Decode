package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "The Gas Blue", group = "Teleop")
public class TheGasBlue extends LinearOpMode {

    private static final int[] TARGET_TAGS = {20, 24};
    private static final String MOTOR_NAME = "turret_motor";

    double speedMultiplier = 0.3;
    private final Intake intake = new Intake();

    @Override
    public void runOpMode() {
        // --- DRIVETRAIN HARDWARE ---
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        Servo latch = hardwareMap.get(Servo.class, "latchServo");

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        DcMotorEx[] driveMotors = {leftFront, leftBack, rightFront, rightBack};
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- SUBSYSTEMS ---
        Turret turretSubsystem = new Turret();
        turretSubsystem.init(hardwareMap, MOTOR_NAME); // Fixed method call

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionManager visionManager = new VisionManager(hardwareMap, cam, new Size(640, 480));

        intake.init(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, telemetry);

        waitForStart();
        shooter.setMode(Shooter.Mode.RAW);
        latch.setPosition(0);

        while (opModeIsActive()) {
            // --- DRIVE LOGIC ---
            double leftX = gamepad1.left_stick_x;
            double leftY = gamepad1.left_stick_y;
            double rightX = -gamepad1.right_stick_x;

            double frontLeftPower = leftY - leftX + rightX;
            double rearLeftPower = leftY + leftX + rightX;
            double frontRightPower = leftY + leftX - rightX;
            double rearRightPower = leftY - leftX - rightX;

            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(rearLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower; rearLeftPower /= maxPower;
                frontRightPower /= maxPower; rearRightPower /= maxPower;
            }

            speedMultiplier = 0.3 + (0.7 * gamepad1.right_trigger);
            leftFront.setPower(frontLeftPower * speedMultiplier);
            leftBack.setPower(rearLeftPower * speedMultiplier);
            rightFront.setPower(frontRightPower * speedMultiplier);
            rightBack.setPower(rearRightPower * speedMultiplier);

            // --- SHOOTER & INTAKE ---
            if (gamepad2.a) {
                shooter.setRaw(1);
                if (shooter.getVelocity() > 900) {
                    latch.setPosition(0.3);
                    intake.runIntake();
                }
            } else {
                shooter.setRaw(gamepad1.left_trigger); // Analog control on trigger
                intake.stopIntake();
                latch.setPosition(0);
            }

            if (gamepad1.right_bumper){
                intake.runIntake();
            } else if (gamepad1.left_bumper){
                intake.runOutTake();
                shooter.setRaw(-.5);
            }

            AprilTagDetection target = null;
            for (int id : TARGET_TAGS) {
                target = visionManager.getTargetDetection(id);
                if (target != null) break; // Priority: If it finds 20, it stops looking and tracks it.
            }

            // --- TURRET & VISION TRACKING ---
            // Priority: Manual control on stick, otherwise auto-track
            if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                turretSubsystem.setManualPower(gamepad2.right_stick_x * .9);
            } else {
                // If no manual input, the turret will track whichever tag was found
                turretSubsystem.updateTurretTracking(target, getRuntime());
            }

            // --- TELEMETRY ---
            if (target != null) {
                telemetry.addData("Status", "Tracking Tag %d", target.id);
                telemetry.addData("Bearing Error", "%.2f", -target.ftcPose.bearing);
            } else {
                telemetry.addLine("Searching for Tags 20 or 24...");
            }

            shooter.periodic(target);
            telemetry.addData("Turret Pos", turretSubsystem.getPosition());
            telemetry.addData("Flywheel Vel", shooter.getVelocity());
            telemetry.update();
        }
    }
}