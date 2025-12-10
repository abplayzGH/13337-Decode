package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.VisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class Shooter {

    /* ---------------- Mode ---------------- */
    public enum Mode { RAW, FIXED, DYNAMIC }

    /* ---------------- Tunable Config Values ---------------- */
    public static double kP = 0.007;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.00045;   // feedforward
    public static double VELO_TOL = 50;
    public static double IDLE_VELO = -600;

    public static boolean tuning = false;
    public static Mode mode = Mode.RAW;

    public static double targetVelocity = 0;
    public static double targetPower = 0;

    /* ---------------- Hardware ---------------- */
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final Servo latch;
    private final VoltageSensor battery;

    private final VisionManager vision;

    private final PIDFController pid = new PIDFController(kP, kI, kD, 0);
    private final InterpLUT distToVelo;

    private final Telemetry telemetry;

    /* ---------------- Constructor ---------------- */
    public Shooter(HardwareMap hw, Telemetry tele) {
        this.telemetry = tele;

        left = hw.get(DcMotorEx.class, "leftFlyWheel");
        right = hw.get(DcMotorEx.class, "rightFlyWheel");
        latch = hw.get(Servo.class, "latchServo");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        vision = new VisionManager(hw, hw.get(WebcamName.class, "Webcam 1"), new Size(640, 480));

        battery = hw.voltageSensor.iterator().next();

        pid.setTolerance(VELO_TOL);

        distToVelo = buildLUT();
    }

    /* ---------------- LUT Setup ---------------- */
    private InterpLUT buildLUT() {
        InterpLUT lut = new InterpLUT();
        lut.add(-100, 0);
        lut.add(0, -970);
        lut.add(2, -990);
        lut.add(3, -1000);
        lut.add(4, -1050);
        lut.add(5, -1050);
        lut.add(6, -1240);
        lut.add(7, -1260);
        lut.add(9, -1440);
        lut.add(11, -1490);
        lut.add(12, -1525);
        lut.add(100, -1580);
        lut.build();
        return lut;
    }

    /* ---------------- Periodic Update ---------------- */
    public void periodic() {
        switch (mode) {
            case RAW:
                applyRaw();
                break;
            case FIXED:
                applyVelocity(targetVelocity);
                break;
            case DYNAMIC:
                applyDynamic();
                break;
        }
        log();
    }

    /* ---------------- Public Control ---------------- */
    public void setRaw(double p) {
        mode = Mode.RAW;
        targetPower = p;
    }

    public void setVelocity(double v) {
        mode = Mode.FIXED;
        targetVelocity = v;
    }

    public void setIdle() {
        setVelocity(IDLE_VELO);
    }

    public boolean isAtTargetVelocity() {
        return pid.atSetPoint();
    }

    /* ---------------- Control Modes ---------------- */

    /** RAW MODE: direct motor power */
    private void applyRaw() {
        left.setPower(targetPower);
        right.setPower(targetPower);
    }

    /** FIXED MODE: hold a target RPM using PIDF+FF */
    private void applyVelocity(double target) {

        if (tuning) {
            pid.setPIDF(kP, kI, kD, 0);
            pid.setTolerance(VELO_TOL);
        }

        double measured = left.getVelocity();
        double ff = kV * target * voltageCompFactor();

        double output = pid.calculate(measured, target) + ff;

        left.setPower(output);
        right.setPower(output);
    }

    /** DYNAMIC MODE: use Apriltag distance to determine velocity */
    private void applyDynamic() {
        List<AprilTagDetection> detections = vision.getDetections();

        if (detections == null || detections.isEmpty()) {
            // No tag → hold idle or power off
            telemetry.addLine("No Tag");
//            setIdle();
            return;
        }

        AprilTagDetection tag = detections.get(0);

        // Use horizontal + forward distance
        double distanceFt = Math.hypot(tag.ftcPose.x, tag.ftcPose.z);

        distanceFt = Range.clip(distanceFt, 1.0, 12.0);

        double target = distToVelo.get(distanceFt);

        applyVelocity(target);
    }

    /* ---------------- Voltage Compensation ---------------- */
    private double voltageCompFactor() {
        // Normal battery voltage range 12.0–13.0 V
        double nominal = 13.0;
        double current = battery.getVoltage();
        return nominal / current;
    }

    /* ---------------- Logging ---------------- */
    private void log() {
        telemetry.addLine("== Shooter ==");
        telemetry.addData("Mode", mode);
        telemetry.addData("Target Velo", targetVelocity);
        telemetry.addData("Measured", left.getVelocity());
        telemetry.addData("Power L", left.getPower());
        telemetry.addData("Power R", right.getPower());
        telemetry.addData("Battery", battery.getVoltage());
    }

    /* ====================================================
                        PIDF Controller
       ==================================================== */
    private static class PIDFController {

        double kP, kI, kD, kF;
        double tolerance = 0;
        double integral = 0;
        double prevError = 0;

        public PIDFController(double p, double i, double d, double f) {
            setPIDF(p, i, d, f);
        }

        public void setPIDF(double p, double i, double d, double f) {
            kP = p;
            kI = i;
            kD = d;
            kF = f;
        }

        public void setTolerance(double t) { tolerance = t; }

        public double calculate(double measured, double target) {
            double error = target - measured;

            integral += error;
            integral = Range.clip(integral, -5000, 5000);   // anti-windup

            double derivative = error - prevError;
            prevError = error;

            return kP * error + kI * integral + kD * derivative + kF * target;
        }

        public boolean atSetPoint() {
            return Math.abs(prevError) <= tolerance;
        }
    }

    /* ====================================================
                        Interpolation LUT
       ==================================================== */
    private static class InterpLUT {
        private final java.util.List<Double> xs = new java.util.ArrayList<>();
        private final java.util.List<Double> ys = new java.util.ArrayList<>();

        public void add(double x, double y) {
            xs.add(x);
            ys.add(y);
        }

        public void build() {
            for (int i = 0; i < xs.size() - 1; i++) {
                for (int j = i + 1; j < xs.size(); j++) {
                    if (xs.get(j) < xs.get(i)) {
                        double tx = xs.get(i);
                        double ty = ys.get(i);
                        xs.set(i, xs.get(j));
                        ys.set(i, ys.get(j));
                        xs.set(j, tx);
                        ys.set(j, ty);
                    }
                }
            }
        }

        public double get(double x) {
            if (x <= xs.get(0)) return ys.get(0);
            if (x >= xs.get(xs.size() - 1)) return ys.get(ys.size() - 1);

            for (int i = 0; i < xs.size() - 1; i++) {
                double x0 = xs.get(i);
                double x1 = xs.get(i + 1);

                if (x >= x0 && x <= x1) {
                    double y0 = ys.get(i);
                    double y1 = ys.get(i + 1);
                    double t = (x - x0) / (x1 - x0);
                    return y0 + t * (y1 - y0);
                }
            }
            return ys.get(0);
        }
    }
}
