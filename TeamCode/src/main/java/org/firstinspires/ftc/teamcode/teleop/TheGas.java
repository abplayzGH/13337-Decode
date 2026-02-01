package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.vision.opencv.ColorRange;
@Config
@TeleOp(name = "The Gas", group = "Teleop")
public class TheGas extends LinearOpMode {

    private static final int[] TARGET_TAGS = {20, 24};
    private static final String MOTOR_NAME = "turret_motor";

    private static final double SHOOTER_READY_VELOCITY = 700;
    private static final double LATCH_OPEN = 0.15;
    private static final double LATCH_CLOSED = 0;

    public FtcDashboard dashboard;

    public Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() {
        //TODO Make subsystem robot class
        //TODO Make this in to a state machine
        //TODO Add better error handling
        //TODO Optimize
        Servo latch = hardwareMap.get(Servo.class, "latchServo");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        /* ---------------- SUBSYSTEMS ---------------- */
        Intake intake = new Intake();
        intake.init(hardwareMap);

        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        Turret turret = new Turret();
        turret.init(hardwareMap, MOTOR_NAME);

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480)); //TODO Tune webcam

        Mecanum mecanum = new Mecanum();
        mecanum.Init(hardwareMap);

        waitForStart();

        latch.setPosition(0);
//        vision.startDashboardStream(15);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {

            /* -------- DRIVE -------- */
            // Get joystick inputs
            // X = gamepad1 = wheel movement
            double leftX = gamepad1.left_stick_x; // Forward/Backward
            double leftY = gamepad1.left_stick_y;  // Left/Right
            double rightX = -gamepad1.right_stick_x; // Rotate
            double speed = 0.3 + 0.7 * gamepad1.right_trigger;

            mecanum.Drive(leftX, leftY, rightX, speed);

            /* -------- SHOOTER / INTAKE -------- */
            boolean shootRaw = gamepad2.a || gamepad1.a;
            boolean shootDynamic = gamepad2.b || gamepad1.b;

            boolean intakeIn = gamepad1.right_bumper || gamepad2.right_bumper;
            boolean intakeOut = gamepad1.left_bumper || gamepad2.left_bumper;

            //TODO Add vibration feedback for shooter ready
            //TODO Add vibration for end game
            //TODO Add color sensor feedback for intake
            //TODO Test dynamic shooting

            float[] hsv = new float[3];
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);

            float hue = hsv[0]; // Hue is measured in degrees (0-360)

            /* -------- VISION / TURRET -------- */
            AprilTagDetection target = null; //TODO Test edge cases with no target
            for (int id : TARGET_TAGS) {
                target = vision.getTargetDetection(id);
                if (target != null) break;
            }


            //TODO Test turret tracking``
            if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                turret.setManualPower(gamepad2.right_stick_x * 0.8);
            } else if (target != null){
                turret.updateTurretTracking(target, getRuntime()); //TODO Add angle limit to auto tracking
            } else {
                turret.setManualPower(0);
            }

            shooter.periodic(target);


            if (shootRaw) {
                shooter.setMode(Shooter.Mode.FIXED);
                shooter.setTargetVelocity(SHOOTER_READY_VELOCITY);
                if (shooter.isAtTargetVelocity()) {
                    latch.setPosition(LATCH_OPEN);
                    intake.runIntake();
                    intake.runTransfer();
                }
            } else if (shootDynamic) {
                shooter.setMode(Shooter.Mode.DYNAMIC);
                if (shooter.isAtTargetVelocity()) {
                    latch.setPosition(LATCH_OPEN);
                    intake.runIntake();
                    intake.runTransfer();

                }
            } else if (gamepad2.right_trigger >= .05){
                shooter.setMode(Shooter.Mode.RAW);
                shooter.setRaw(gamepad2.right_trigger);
                if (gamepad2.left_bumper) {
                    latch.setPosition(LATCH_OPEN);
                    intake.runIntake();
                    intake.runTransfer();
                }
            } else if (intakeOut) {
                shooter.setRaw(-0.5);
                intake.runOutTake();
                latch.setPosition(LATCH_OPEN);
            } else if (intakeIn) {
                shooter.setRaw(0);
                intake.runIntake();
                if (!(hue > 145 && hue < 205)) {
                    telemetry.addLine("Transferring");
                    intake.runTransfer();
                }
                latch.setPosition(LATCH_CLOSED);
            } else {
                shooter.setRaw(0);
                intake.stopIntake();
                latch.setPosition(LATCH_CLOSED);
//                shooter.setIdle();
            }


            /* -------- TELEMETRY -------- */
//            telemetry.addData("Shooter Vel", shooter.getVelocity());
            telemetry.addData("Turret Pos", turret.getPosition());
            telemetry.addData("Tag", target != null ? target.id : "None");
            telemetry.addData("Servo Position", latch.getPosition());
            telemetry.addData("Color", colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
            telemetry.addData("Hue", hue);
            telemetry.update();
            dashboardTelemetry.update();

        }
    }
}
