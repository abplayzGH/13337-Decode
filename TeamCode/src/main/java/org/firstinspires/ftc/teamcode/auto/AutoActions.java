package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

public class AutoActions {
    private final Robot robot;
    private final Shooter shooter;
    private final Intake intake;
    private final Servo latch;

    public AutoActions(){
        robot = Robot.get();
        shooter = robot.shooter;
        intake = robot.intake;
        latch = robot.latch;
    }

    public Action spinUp() {
        return new Action() {
            final ElapsedTime timer = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    timer.reset();
                    started = true;
                }

                shooter.setMode(Shooter.Mode.FIXED);
                shooter.setTargetVelocity(Robot.SHOOTER_READY_VELOCITY);
                // Explicitly pass null when no vision data is available
                shooter.periodic(null);

                // Publish helpful telemetry for tuning/debugging
                try {
                    packet.put("shooter_target_rpm", Robot.SHOOTER_READY_VELOCITY);
                    packet.put("shooter_rpm", shooter.getVelocity());
                    packet.put("spinup_elapsed_s", timer.seconds());
                } catch (Exception ignored) {}

                boolean timeout = timer.seconds() > 3;
                return !(shooter.isAtTargetVelocity() || timeout);
            }
        };
    }

    public Action fire() {
        return packet -> {
            latch.setPosition(Robot.LATCH_OPEN);
            intake.runTransfer();
            intake.runIntake();
            return false;
        };
    }

    public Action intake() {
        return packet -> {
            shooter.setRaw(0);
            intake.runIntake();

            if (robot.ranger.getDistance() < 10) {
                intake.runTransfer();
            }

            latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

    public Action stopShooter() {
        return packet -> {
            robot.reset();
            latch.setPosition(Robot.LATCH_CLOSED);
            return false;
        };
    }

}
