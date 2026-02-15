package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Config
@TeleOp(name = "The Gas", group = "Teleop")
public class TheGas2 extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = Robot.get().Init(Robot.Mode.TELEOP, hardwareMap, telemetry);

        // ✅ Persistent state (does NOT reset every loop)
        Robot.RobotState state = Robot.RobotState.IDLE;
        if (robot.limelight != null) robot.limelight.getAprilTags();
        waitForStart();

        while (opModeIsActive()) {

            /* ================= DRIVE ================= */

            double forward = -gamepad1.left_stick_y;   // Forward/back
            double strafe  =  gamepad1.left_stick_x;   // Left/right
            double rotate  = -gamepad1.right_stick_x;  // Turn
            double speed   = 0.3 + 0.7 * gamepad1.right_trigger;

            robot.mecanumTeleop.Drive(strafe, forward, rotate, speed);

            /* ================= INPUTS ================= */

            boolean shootFixed   = gamepad2.a || gamepad1.a;
            boolean shootDynamic = gamepad2.b || gamepad1.b;
            boolean intakeIn     = gamepad1.right_bumper || gamepad2.right_bumper;
            boolean intakeOut    = gamepad1.left_bumper  || gamepad2.left_bumper;

            /* ================= SAFE VISION ================= */

            double tagDistance = 0;
            double tagX = 0;
            boolean hasTarget = false;
            //Update tags
            if (robot.limelight != null) {
                robot.limelight.getAprilTags();
            }

            if (robot.limelight != null && robot.limelight.hasValidTarget()) {
                tagDistance = robot.limelight.getTagDistance();
                tagX = robot.limelight.getTagLocationX();
                hasTarget = true;
            }

            // Always call shooter.periodic and turret update so they can handle lost/seen targets
            robot.shooter.periodic(hasTarget ? tagDistance : null);
            robot.turret.updateTrackingLimelight(tagX, hasTarget);


            /* ================= TUNE SHOOTER ================= */

            if (gamepad1.dpad_up) {
                Robot.SHOOTER_READY_VELOCITY += 10;
            } else if (gamepad1.dpad_down) {
                Robot.SHOOTER_READY_VELOCITY -= 10;
            }

            /* ================= STATE TRANSITIONS ================= */

            switch (state) {
                case IDLE:
                    if (shootFixed) {
                        state = Robot.RobotState.SPINUP_FIXED;
                    } else if (shootDynamic) {
                        state = Robot.RobotState.SPINUP_DYNAMIC;
                    } else if (intakeIn) {
                        state = Robot.RobotState.INTAKING;
                    } else if (intakeOut) {
                        state = Robot.RobotState.OUTTAKING;
                    }
                    break;

                case INTAKING:
                    if (!intakeIn) {
                        state = Robot.RobotState.IDLE;
                    }
                    break;

                case OUTTAKING:
                    if (!intakeOut) {
                        state = Robot.RobotState.IDLE;
                    }
                    break;

                case SPINUP_FIXED:
                    if (!shootFixed) {
                        state = Robot.RobotState.IDLE;
                    } else if (robot.shooter.isAtTargetVelocity()) {
                        state = Robot.RobotState.SHOOTING;
                    }
                    break;

                case SPINUP_DYNAMIC:
                    if (!shootDynamic) {
                        state = Robot.RobotState.IDLE;
                    } else if (robot.shooter.isAtTargetVelocity()) {
                        state = Robot.RobotState.SHOOTING;
                    }
                    break;

                case SHOOTING:
                    if (!shootFixed && !shootDynamic) {
                        state = Robot.RobotState.IDLE;
                    }
                    break;
            }

            /* ================= STATE ACTIONS ================= */

            Shooter.Mode mode = Shooter.Mode.RAW;

            switch (state) {

                case IDLE:
                    robot.intake.stopIntake();
                    robot.latch.setPosition(Robot.LATCH_CLOSED);
                    robot.shooter.setIdle();
                    break;

                case INTAKING:
                    robot.shooter.setRaw(0);
                    robot.intake.runIntake();

                    if (robot.ranger.getDistance() < 10) {
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
                    mode = Shooter.Mode.FIXED;
                    robot.shooter.setTargetVelocity(Robot.SHOOTER_READY_VELOCITY);
                    break;

                case SPINUP_DYNAMIC:
                    mode = Shooter.Mode.DYNAMIC;
                    break;

                case SHOOTING:
                    mode = shootFixed ? Shooter.Mode.FIXED : Shooter.Mode.DYNAMIC;

                    robot.latch.setPosition(Robot.LATCH_OPEN);
                    robot.intake.runIntake();
                    robot.intake.runTransfer();
                    break;
            }

            // Only changes internally if different
            robot.shooter.setMode(mode);

            /* ================= TELEMETRY ================= */

            robot.flightRecorder.addData("State", state);
            robot.flightRecorder.addData("Turret Pos", robot.turret.getPosition());
            robot.flightRecorder.addData("Tag", hasTarget ? robot.limelight.getTagID() : "None");
            robot.flightRecorder.addData("Velocity", robot.shooter.getVelocity());
            robot.flightRecorder.update();
        }
    }
}
