package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.Robot.LATCH_OPEN;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

public class AutoActions {
    private final Robot robot;
    private final Shooter shooter;
    private final Intake intake;
    private final Servo latch;
    double tagArea = 0;
    double tagX = 0;
    boolean hasTarget = false;

    public AutoActions(Robot r){
        robot = r;
        shooter = robot.shooter;
        intake = robot.intake;
        latch = robot.latch;
    }

//    public Action spinUp() {
//        return new Action() {
//            final ElapsedTime timer = new ElapsedTime();
//            boolean started = false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!started) {
//                    timer.reset();
//                    started = true;
//                }
//
//                shooter.setMode(Shooter.Mode.DYNAMIC);
////                shooter.setTargetVelocity(Robot.SHOOTER_READY_VELOCITY);
//                // Explicitly pass null when no vision data is available
//                shooter.periodic(tagArea);
//
//                // Publish helpful telemetry for tuning/debugging
//                try {
//                    packet.put("shooter_target_rpm", Robot.SHOOTER_READY_VELOCITY);
//                    packet.put("shooter_rpm", shooter.getVelocity());
//                    packet.put("spinup_elapsed_s", timer.seconds());
//                } catch (Exception ignored) {}
//
//                boolean timeout = timer.seconds() > 3;
//                return !(shooter.isAtTargetVelocity() || timeout);
//            }
//        };
//    }

    public Action updateLL(){
        return packet -> {

            //Update tags
            if (robot.limelight != null) {
                robot.limelight.getAprilTags();
            }
            hasTarget = false;

            if (robot.limelight != null && robot.limelight.hasValidTarget()) {
                if (robot.limelight.getTagID() == 24 || robot.limelight.getTagID() == 20){
                    robot.flightRecorder.addData("Tag", robot.limelight.getTagID());
                    tagArea = robot.limelight.getTagArea();
                    tagX = robot.limelight.getTagLocationX();

                    hasTarget = true;
                }
            }
            return true;
        };
    }


    public Action fire(double rpm) {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    timer.reset();
                    started = true;
                }

//                shooter.setMode(Shooter.Mode.DYNAMIC);
                shooter.setMode(Shooter.Mode.FIXED);
                shooter.setTargetVelocity(rpm);

                if (shooter.isAtTargetVelocity()) {
                    latch.setPosition(LATCH_OPEN);
                    intake.runIntake();
                    intake.runTransfer();
                }
                // Publish helpful telemetry for tuning/debugging
                try {
                    packet.put("shooter_target_rpm", Robot.SHOOTER_READY_VELOCITY);
                    packet.put("shooter_rpm", shooter.getVelocity());
                    packet.put("spinup_elapsed_s", timer.seconds());
                } catch (Exception ignored) {}

                boolean timeout = timer.seconds() > 2;
                return !(timeout);
            }
        };
    }

    public Action intake() {
        return packet -> {
            latch.setPosition(Robot.LATCH_CLOSED);

//            shooter.setRaw(0);
            intake.runIntake();

            if (robot.ranger.getDistance() < 10) {
                intake.runTransfer();
            }
            return false; // keep running?
        };
    }

    public Action stopIntake() {
        return packet -> {
//            shooter.setRaw(0);
            intake.stopIntake();
            robot.latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

    public Action stopShooter() {
        return packet -> {
            robot.reset();
            shooter.setMode(Shooter.Mode.FIXED);
            shooter.setTargetVelocity(0);
            intake.stopIntake();
            latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

    public Action shooterIdle() {
        return packet -> {
            shooter.setIdle();
            robot.latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

    // Change Pose2d to MecanumDriveRR
//    public Action turretTrack(MecanumDriveRR drive, Vector2d target) {return packet -> {
//        // Now it gets the fresh pose every loop
//        robot.turret.updateFieldCentric(drive.localizer.getPose(), target);
//        return true; };
//        }
        public Action turretTrack() {
            return packet -> {
                robot.turret.updateTrackingLimelight(tagX, hasTarget);
                return true; // keep running forever
            };
        }

    public Action updateShooter() {
        return packet -> {
            shooter.periodic(tagArea);
            return true; // keep running forever
        };
    }


}
