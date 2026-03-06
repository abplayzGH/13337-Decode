package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class TurretV2 {

    private DcMotorEx turretMotor;
    private final ElapsedTime timer = new ElapsedTime();

    /* ---------------- CONTROL CONSTANTS ---------------- */

    public static double kP = 0.006;
    public static double kD = 0.005;
    public static double kF = 0.01;

    public static double MAX_AUTO_POWER = 0.6;

    public static double DEADZONE_DEG = 0.5;

    public static int LEFT_LIMIT = -1000;
    public static int RIGHT_LIMIT = 1000;

    /* ---------------- HOME POSITION ---------------- */

    public static int HOME_POSITION = 0;
    public static double HOME_kP = 0.004;

    /* ---------------- FIELD TARGET ---------------- */

    public static Vector2d GOAL_POS = new Vector2d(-36, 30);

    /* ---------------- TURRET CONVERSION ---------------- */

    public static double TICKS_PER_RADIAN = 325;

    /* ---------------- MOTION PREDICTION ---------------- */

    public static double PROJECTILE_SPEED = 15.0;
    public static double SHOOTER_DELAY = 0.12;

    /* ---------------- STATE ---------------- */

    private double lastError = 0.0;
    private double lastTime = 0.0;
    private double lastTargetSeenTime = 0.0;

    public static double TARGET_LOST_DELAY = 0.4;

    /* ---------------- INIT ---------------- */

    public void init(HardwareMap hardwareMap) {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");

        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();
        lastTime = timer.seconds();
    }

    public void reset() {
        resetController(timer.seconds());
        turretMotor.setPower(0);
    }

    /* ---------------- SOFT LIMITS ---------------- */

    private double limitPower(double power) {

        int pos = turretMotor.getCurrentPosition();

        if (pos <= LEFT_LIMIT && power < 0) return 0;
        if (pos >= RIGHT_LIMIT && power > 0) return 0;

        return power;
    }

    /* ---------------- MANUAL CONTROL ---------------- */

    public void setManualPower(double power) {
        turretMotor.setPower(limitPower(power));
    }

    /* ---------------- RETURN HOME ---------------- */

    public void returnToHome() {

        int pos = turretMotor.getCurrentPosition();
        double error = HOME_POSITION - pos;

        if (Math.abs(error) < 5) {
            turretMotor.setPower(0);
            return;
        }

        double power = Range.clip(error * HOME_kP, -0.4, 0.4);

        turretMotor.setPower(limitPower(power));
    }

    /* ---------------- LIMELIGHT TRACKING ---------------- */

    public void updateTrackingLimelight(double tx, boolean targetVisible) {

        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;

        if (dt <= 0.01) return;

        if (targetVisible) {
            lastTargetSeenTime = currentTime;
        }

        if (!targetVisible) {

            if (currentTime - lastTargetSeenTime > TARGET_LOST_DELAY) {
                returnToHome();
            } else {
                turretMotor.setPower(0);
            }

            return;
        }

        double error = tx;

        if (Math.abs(error) < DEADZONE_DEG) {

            turretMotor.setPower(0);

            lastError = error;
            lastTime = currentTime;

            return;
        }

        double derivative = Range.clip((error - lastError) / dt, -50, 50);

        double output = (kP * error) + (kD * derivative) + (Math.signum(error) * kF);

        double power = Range.clip(output, -MAX_AUTO_POWER, MAX_AUTO_POWER);

        turretMotor.setPower(limitPower(power));

        lastError = error;
        lastTime = currentTime;
    }

    /* ---------------- FIELD AIMING ---------------- */

    public double getAngleToGoal(Pose2d robotPose) {

        double dx = GOAL_POS.x - robotPose.position.x;
        double dy = GOAL_POS.y - robotPose.position.y;

        return Math.atan2(dy, dx);
    }

    public double getTurretTargetAngle(Pose2d robotPose) {

        double goalAngle = getAngleToGoal(robotPose);

        return goalAngle - robotPose.heading.toDouble();
    }

    public int angleToTicks(double angle) {
        return (int)(angle * TICKS_PER_RADIAN);
    }

    /* ---------------- MOTION PREDICTION ---------------- */

    public Pose2d predictRobotPose(Pose2d pose, PoseVelocity2d velocity, double dt) {

        double newX = pose.position.x + velocity.linearVel.x * dt;
        double newY = pose.position.y + velocity.linearVel.y * dt;

        double newHeading = pose.heading.toDouble() + velocity.angVel * dt;

        return new Pose2d(newX, newY, newHeading);
    }

    public double getProjectileTime(Pose2d robotPose) {

        double dx = GOAL_POS.x - robotPose.position.x;
        double dy = GOAL_POS.y - robotPose.position.y;

        double distance = Math.hypot(dx, dy);

        return (distance / PROJECTILE_SPEED) + SHOOTER_DELAY;
    }

    /* ---------------- PREDICTIVE TRACKING ---------------- */

    public void trackGoalPredicted(Pose2d robotPose, PoseVelocity2d vel) {

        double projectileTime = getProjectileTime(robotPose);

        Pose2d predictedPose = predictRobotPose(robotPose, vel, projectileTime);

        double dx = GOAL_POS.x - predictedPose.position.x;
        double dy = GOAL_POS.y - predictedPose.position.y;

        double goalAngle = Math.atan2(dy, dx);

        double turretAngle = goalAngle - predictedPose.heading.toDouble();

        int targetTicks = angleToTicks(turretAngle);

        int pos = turretMotor.getCurrentPosition();

        double error = targetTicks - pos;

        double power = Range.clip(error * kP, -MAX_AUTO_POWER, MAX_AUTO_POWER);

        turretMotor.setPower(limitPower(power));
    }

    /* ---------------- UTILITY ---------------- */

    private void resetController(double time) {
        lastError = 0;
        lastTime = time;
    }

    public int getPosition() {
        return turretMotor.getCurrentPosition();
    }
}