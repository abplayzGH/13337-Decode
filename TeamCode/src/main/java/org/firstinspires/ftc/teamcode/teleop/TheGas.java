package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@TeleOp(name = "The Gas", group = "Teleop")
public class TheGas extends LinearOpMode {

    private static final int[] TARGET_TAGS = {20, 24};
    private static final String MOTOR_NAME = "turret_motor";

    private static final double SHOOTER_READY_VELOCITY = 900;

    @Override
    public void runOpMode() {


        Servo latch = hardwareMap.get(Servo.class, "latchServo");

        /* ---------------- SUBSYSTEMS ---------------- */
        Intake intake = new Intake();
        intake.init(hardwareMap);

        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        Turret turret = new Turret();
        turret.init(hardwareMap, MOTOR_NAME);

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480));

        FieldCentricDrive drive = new FieldCentricDrive();
        drive.init(hardwareMap);

        waitForStart();

        latch.setPosition(0);

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {

            /* -------- DRIVE -------- */
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            double speed = 0.3 + 0.7 * gamepad1.right_trigger;

            drive.drive(x, y, rx, speed);

            if (gamepad1.y) {
                drive.resetHeading();
            }

            /* -------- SHOOTER / INTAKE -------- */
            boolean shootRaw = gamepad2.a;
            boolean intakeIn = gamepad1.right_bumper;
            boolean intakeOut = gamepad1.left_bumper;
            boolean shootDynamic = gamepad2.b;


            if (shootRaw) {
                shooter.setMode(Shooter.Mode.RAW);
                shooter.setRaw(1);
                if (shooter.getVelocity() > SHOOTER_READY_VELOCITY) {
                    latch.setPosition(0.3);
                    intake.runIntake();
                }
            } else if (shootDynamic) {
                shooter.setMode(Shooter.Mode.DYNAMIC);
                if (shooter.isAtTargetVelocity()) {
                    latch.setPosition(0.3);
                    intake.runIntake();
                }
            } else if (intakeOut) {
                shooter.setRaw(-0.5);
                intake.runOutTake();
                latch.setPosition(0.3);
            } else if (intakeIn) {
                shooter.setRaw(0);
                intake.runIntake();
                latch.setPosition(0);
            } else {
                shooter.setRaw(0);
                intake.stopIntake();
                latch.setPosition(0);
            }

            /* -------- VISION / TURRET -------- */
            AprilTagDetection target = null;
            for (int id : TARGET_TAGS) {
                target = vision.getTargetDetection(id);
                if (target != null) break;
            }

            if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                turret.setManualPower(gamepad2.right_stick_x * 0.9);
            } else {
                turret.updateTurretTracking(target, getRuntime());
            }

            shooter.periodic(target);

            /* -------- TELEMETRY -------- */
            telemetry.addData("Heading", Math.toDegrees(drive.getHeading()));
            telemetry.addData("Shooter Vel", shooter.getVelocity());
            telemetry.addData("Turret Pos", turret.getPosition());
            telemetry.addData("Tag", target != null ? target.id : "None");
            telemetry.update();
        }
    }
}
