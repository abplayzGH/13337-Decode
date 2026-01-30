package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Robot.alliance;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

//import org.firstinspires.ftc.teamcode.mechanisms.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "The Gas", group = "Teleop")
public class TheGas extends LinearOpMode {
    private Robot robot;

    enum ShooterState {
        IDLE,
        INTAKING,
        OUTTAKING,
        SPINUP_FIXED,
        SPINUP_DYNAMIC,
        SHOOTING
    }

    ShooterState shooterState = ShooterState.IDLE;

    @Override
    public void runOpMode() {
        //TODO Make subsystem robot class
        //TODO Make this in to a state machine
        //TODO Add better error handling
        //TODO Optimize

        // Safe to use robot fields now
        robot = Robot.get().Init(Robot.Mode.AUTO, hardwareMap, telemetry);
        waitForStart();

        boolean endgameRumbled = false;

        /* ================= MAIN LOOP ================= */
        while (opModeIsActive()) {

            /* -------- DRIVE -------- */
            // Get joystick inputs
            // X = gamepad1 = wheel movement
            double leftX = gamepad1.left_stick_x; // Forward/Backward
            double leftY = gamepad1.left_stick_y;  // Left/Right
            double rightX = -gamepad1.right_stick_x; // Rotate
            double speed = 0.3 + 0.7 * gamepad1.right_trigger;

            robot.mecanum.Drive(leftX, leftY, rightX, speed);

            /* -------- SHOOTER / INTAKE -------- */
            boolean shootRaw = gamepad2.a || gamepad1.a;
            boolean shootDynamic = gamepad2.b || gamepad1.b;

            boolean intakeIn = gamepad1.right_bumper || gamepad2.right_bumper;
            boolean intakeOut = gamepad1.left_bumper || gamepad2.left_bumper;

            shooterState = (shootRaw) ? ShooterState.SPINUP_FIXED :
                    (shootDynamic) ? ShooterState.SPINUP_DYNAMIC :
                            (intakeIn) ? ShooterState.INTAKING :
                                    (intakeOut) ? ShooterState.OUTTAKING :
                                            ShooterState.IDLE;


            //TODO Add vibration feedback for shooter ready

            float[] hsv = new float[3];
            Color.RGBToHSV(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue(), hsv);

            float hue = hsv[0]; // Hue is measured in degrees (0-360)

            /* -------- VISION / TURRET -------- */
            // Get target once
            AprilTagDetection target = robot.vision.getTargetDetection(Robot.TARGET_TAG);

            // Turret manual or auto tracking
            if (Math.abs(gamepad2.right_stick_x) > 0.05) {
                robot.turret.setManualPower(gamepad2.right_stick_x * 0.7);
            } else if (target != null) {
                robot.turret.updateTracking(target);
            } else {
                robot.turret.setManualPower(0);
            }


            robot.shooter.periodic(target);


            switch (shooterState) {

                case IDLE:
                    robot.shooter.setRaw(0);
                    robot.intake.stopIntake();
                    robot.latch.setPosition(Robot.LATCH_CLOSED);
                    break;

                case INTAKING:
                    robot.shooter.setRaw(0);
                    robot.intake.runIntake();

                    if (!(hue > 145 && hue < 205)) {
                        robot.intake.runTransfer();
                    }
                    robot.latch.setPosition(Robot.LATCH_CLOSED);
                    break;

                case OUTTAKING:
                    robot.shooter.setRaw(-0.5);
                    robot.intake.runOutTake();
                    robot.latch.setPosition(Robot.LATCH_OPEN);
                    break;

                case SPINUP_FIXED:
                    robot.shooter.setMode(Shooter.Mode.FIXED);
                    robot.shooter.setTargetVelocity( Robot.SHOOTER_READY_VELOCITY);

                    if (robot.shooter.isAtTargetVelocity()) {
                        shooterState = ShooterState.SHOOTING;
                    }
                    break;

                case SPINUP_DYNAMIC:
                    robot.shooter.setMode(Shooter.Mode.DYNAMIC);

                    if (robot.shooter.isAtTargetVelocity()) {
                        shooterState = ShooterState.SHOOTING;
                    }
                    break;

                case SHOOTING:
                    robot.latch.setPosition(Robot.LATCH_OPEN);
                    robot.intake.runIntake();
                    robot.intake.runTransfer();
                    break;
            }


            if (!endgameRumbled && robot.matchTimer.seconds() >= 90) {
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                endgameRumbled = true;
            }


            /* -------- TELEMETRY -------- */
//            telemetry.addData("Shooter Vel", shooter.getVelocity());
            robot.flightRecorder.addData("Tag", target != null ? target.id : "None");
            robot.flightRecorder.addData("Hue", hue);
            robot.endLoop();

        }
    }
}
