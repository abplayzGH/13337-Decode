package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDriveRR;

@Config
public class Turret {

    private DcMotorEx turretMotor;
    private final ElapsedTime timer = new ElapsedTime();

    // ---- Control Constants ----
    public static double kP = 0.006;
    public static double kD = 0.005;
    public static double kF = 0.01;      // Feedforward (helps overcome friction)
    public static double MAX_AUTO_POWER = 0.6;
    public static double DEADZONE_DEG = 0.5;
    public static int LEFT_LIMIT = -160;
    public static int RIGHT_LIMIT = 130;
    public static int HOME_POSITION = 0;
    public static double HOME_kP = 0.004;
    private double lastTargetSeenTime = 0;
    public static double TARGET_LOST_DELAY = 0.4;

    // ---- State ----
    private double lastError = 0.0;
    private double lastTime = 0.0;

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer.reset();
        lastTime = timer.seconds();
    }
    public void reset(){
        resetController(0);
        turretMotor.setPower(0);
    }

    // ---- Soft Limits ----
    private double limitPower(double power) {
        int pos = turretMotor.getCurrentPosition();

        // ---- Encoder Limits (TUNE THESE) ----
        if (pos <= LEFT_LIMIT && power < 0) return 0;
        if (pos >= RIGHT_LIMIT && power > 0) return 0;

        return power;
    }

    // ---- Manual Control ----
    public void setManualPower(double power) {
        turretMotor.setPower(limitPower(power));
    }


    public void updateTrackingLimelight(double tx, boolean targetVisible) {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;

        if (dt <= 0.01) {
            return;
        }

        // If no target is seen, stop moving
        if (!targetVisible) {
            resetController(currentTime);
            returnToHome();
            return;
        }

        // tx is the horizontal offset.
        // We want the turret to move until tx is 0.
        double error = tx;

        if (Math.abs(error) < DEADZONE_DEG) {
            turretMotor.setPower(0);
            lastError = error;
            lastTime = currentTime;
            return;
        }

        double derivative = Range.clip((error - lastError) / dt, -50, 50);

        // PD + Feedforward
        double output = (kP * error) + (kD * derivative) + (Math.signum(error) * kF);

        double power = Range.clip(output, -MAX_AUTO_POWER, MAX_AUTO_POWER);

        turretMotor.setPower(limitPower(power));

        lastError = error;
        lastTime = currentTime;

    }

    private void resetController(double time) {
        lastError = 0;
        lastTime = time;
    }

    public double getPosition() {
        // Return a safe value if motor is not initialized to avoid NPEs when debugging
        if (turretMotor == null) {
            return Double.NaN;
        }
        return turretMotor.getCurrentPosition();
    }

    // Diagnostic helper: returns a short, human readable status string for telemetry
    public String getDiagnostics() {
        if (turretMotor == null) return "turretMotor=null";
        String mode = (turretMotor.getMode() != null) ? turretMotor.getMode().toString() : "null";
        String zp = (turretMotor.getZeroPowerBehavior() != null) ? turretMotor.getZeroPowerBehavior().toString() : "null";
        return String.format("pos=%d mode=%s power=%.2f zp=%s", turretMotor.getCurrentPosition(), mode, turretMotor.getPower(), zp);
    }

    // Inside Turret.java
    public void updateFieldCentric(MecanumDriveRR drive, Vector2d targetVec) {
        // 1. Find vector from robot to goal
        double dx = targetVec.x - drive.localizer.getPose().position.x;
        double dy = targetVec.y - drive.localizer.getPose().position.y;

        // 2. Calculate the absolute field angle to the goal
        double angleToGoal = Math.atan2(dy, dx);

        // 3. Subtract robot heading to get angle relative to robot chassis
        double relativeAngle = AngleUnit.normalizeRadians(angleToGoal - drive.localizer.getPose().heading.toDouble());

        // 4. Convert to degrees (if your tx logic uses degrees) and update
        updateTrackingLimelight(Math.toDegrees(relativeAngle), true);
    }

    public void returnToHome() {
        int pos = turretMotor.getCurrentPosition();
        double error = HOME_POSITION - pos;

        // small deadzone so it doesn't jitter
        if (Math.abs(error) < 5) {
            turretMotor.setPower(0);
            return;
        }

        double power = Range.clip(error * HOME_kP, -0.6, 0.6);

        turretMotor.setPower(limitPower(power));
    }

}
