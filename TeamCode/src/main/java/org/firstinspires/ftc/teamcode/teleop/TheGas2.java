package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Robot.LATCH_CLOSED;
import static org.firstinspires.ftc.teamcode.Robot.LATCH_OPEN;

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
//        Robot.RobotState state = Robot.RobotState.IDLE;
        if (robot.limelight != null) robot.limelight.getAprilTags();
        waitForStart();

        while (opModeIsActive()) {

            /* ================= DRIVE ================= */

            double forward = gamepad1.left_stick_y;   // Forward/back
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
                if (robot.limelight.getTagID() == Robot.TARGET_TAG){
                    robot.flightRecorder.addData("Tag", robot.limelight.getTagID());
                    tagDistance = robot.limelight.getTagDistance();
                    tagX = robot.limelight.getTagLocationX();

                    hasTarget = true;
                }
            }

            // Always call shooter.periodic and turret update so they can handle lost/seen targets
            robot.turret.updateTrackingLimelight(tagX, hasTarget);


            /* ================= TUNE SHOOTER ================= */

            if (gamepad1.dpadUpWasPressed()) {
                Robot.SHOOTER_READY_VELOCITY += 10;
            } else if (gamepad1.dpadDownWasPressed()) {
                Robot.SHOOTER_READY_VELOCITY -= 10;
            }

            /* ================= STATE TRANSITIONS ================= */
            if (shootFixed) {
                robot.shooter.setMode(Shooter.Mode.FIXED);
                robot.shooter.setTargetVelocity(Robot.SHOOTER_READY_VELOCITY);
                if (robot.shooter.isAtTargetVelocity()) {
                    robot.latch.setPosition(LATCH_OPEN);
                    robot.intake.runIntake();
                    robot.intake.runTransfer();
                }
            } else if (shootDynamic) {
                robot.shooter.setMode(Shooter.Mode.DYNAMIC);
                if (robot.shooter.isAtTargetVelocity()) {
                    robot.latch.setPosition(LATCH_OPEN);
                    robot.intake.runIntake();
                    robot.intake.runTransfer();

                }
//            } else if (gamepad2.right_trigger >= .05) {
//                robot.shooter.setMode(Shooter.Mode.RAW);
//                robot.shooter.setRaw(gamepad2.right_trigger);
//                if (gamepad2.left_bumper) {
//                    robot.latch.setPosition(LATCH_OPEN);
//                    robot.intake.runIntake();
//                    robot.intake.runTransfer();
//                }
            } else if (intakeOut) {
                robot.shooter.setRaw(-0.5);
                robot.intake.runOutTake();
                robot.latch.setPosition(LATCH_OPEN);
            } else if (intakeIn) {
                robot.shooter.setRaw(0);
                robot.intake.runIntake();
                if (robot.ranger.getDistance() >= .2) {
                    telemetry.addLine("Transferring");
                    robot.intake.runTransfer();
                }
                robot.latch.setPosition(LATCH_CLOSED);
            } else {
                robot.shooter.setRaw(0);
                robot.intake.stopIntake();
                robot.latch.setPosition(LATCH_CLOSED);
//                shooter.setIdle();
            }
            robot.shooter.periodic(tagDistance);


            /* ================= TELEMETRY ================= */

//            robot.flightRecorder.addData("State", state);
            robot.flightRecorder.addData("Ranger", robot.ranger.getDistance());
            robot.flightRecorder.addData("Turret Pos", robot.turret.getPosition());
            robot.flightRecorder.addData("Tag", hasTarget ? robot.limelight.getTagID() : "None");
            robot.flightRecorder.addData("Velocity", robot.shooter.getVelocity());
            robot.flightRecorder.addData("Distance", tagDistance);
            robot.flightRecorder.addData("X", tagX);
            robot.flightRecorder.addData("Pose", robot.limelight.tagPose);
            robot.flightRecorder.addData("Y", robot.limelight.getTagLocationY());
            robot.flightRecorder.addData("Ready Velocity", Robot.SHOOTER_READY_VELOCITY);
            robot.flightRecorder.update();
        }
    }
}
