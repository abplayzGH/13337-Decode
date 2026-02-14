package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
@SuppressWarnings("unused")
@Config
@TeleOp(name = "The Gas", group = "Teleop")
public class TheGas2 extends LinearOpMode {

    private static final int[] TARGET_TAGS = {20, 24};
    public static double SHOOTER_READY_VELOCITY = 1400;
    public static double LATCH_OPEN = 0.1;
    public static double LATCH_CLOSED = 0;

    @Override
    public void runOpMode() {
        //TODO Make subsystem robot class
        //TODO Make this in to a state machine
        //TODO Add better error handling
        //TODO Optimize

        Robot robot = Robot.get().Init(Robot.Mode.TELEOP, hardwareMap, telemetry);
        Servo latch = hardwareMap.get(Servo.class, "latchServo");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /* ---------------- SUBSYSTEMS ---------------- */
        Intake intake = new Intake();
        intake.init(hardwareMap);

        Shooter shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        Turret turret = new Turret();
        turret.init(hardwareMap);

//        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        VisionManager vision = new VisionManager(hardwareMap, cam, new Size(640, 480)); //TODO Tune webcam

        Mecanum mecanum = new Mecanum();
        mecanum.Init(hardwareMap);

        waitForStart();

        latch.setPosition(LATCH_CLOSED);
//        vision.startDashboardStream(15);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        limelight.start();

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

            Pose3D botPose;

            /* -------- VISION / TURRET -------- */
            LLResult result = limelight.getLatestResult();
            boolean targetFound = false;
            double targetTx = 0;
            Double currentTA = null;


            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                // Look for your specific tags (20, 24)

                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    targetFound = true;
                    targetTx = fr.getTargetXDegrees();
                    currentTA = fr.getTargetArea();
                    break;

                }
            }


// Pass TA to the shooter
            shooter.periodic(currentTA);
            turret.updateTrackingLimelight(targetTx, targetFound);
// Control logic
//            if (Math.abs(gamepad2.right_stick_x) > 0.05) {
//                turret.setManualPower(gamepad2.right_stick_x * 0.3);
//            } else {
//                // This will either track the tag or stop the motor if targetFound is false
//                turret.updateTrackingLimelight(targetTx, targetFound);
//            }


            if (result != null) {
                if (result.isValid()) {
                    botPose = result.getBotpose();

                    telemetry.addData("Tag", result.getFiducialResults());
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botPose.toString());
                    telemetry.addData("Id", result.getDetectorResults());
                    telemetry.addData("Id", result.getClassifierResults());
                }
            }

            if (gamepad1.dpad_up){
                SHOOTER_READY_VELOCITY += 25;
            } else if (gamepad1.dpad_down){
                SHOOTER_READY_VELOCITY -= 25;
            }


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
            } else if (gamepad2.right_trigger >= .05) {
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
//                    telemetry.addData("Tag", target != null ? target.id : "None");
            telemetry.addData("Servo Position", latch.getPosition());
            telemetry.addData("Color", colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
            telemetry.addData("Hue", hue);
            telemetry.addData("Velocity", shooter.getVelocity());
            telemetry.addData("Target Ta", currentTA != null ? currentTA : "None");
            telemetry.addData("Target Tx", targetTx);
            telemetry.update();
            dashboardTelemetry.update();

        }
    }
}