//package org.firstinspires.ftc.teamcode.mechanisms;
//
//import android.util.Size;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.vision.VisionManager;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//import java.util.List;
//
//public class Shooter {
//
//    /* ---------------- Mode ---------------- */
//    public enum Mode { RAW, FIXED, DYNAMIC }
//
//    /* ---------------- Tunable Config Values ---------------- */
//    public static double kP = 0.007;
//    public static double kI = 0.0;
//    public static double kD = 0.0;
//    public static double kV = 0.00045;   // feedforward
//    public static double VELO_TOL = 50;
//    public static double IDLE_VELO = 600;
//
//    public static boolean tuning = false;
//    public static Mode mode = Mode.RAW;
//
//    public static double targetVelocity = 1000;
//    public static double targetPower = 0;
//
//    /* ---------------- Hardware ---------------- */
//    private final DcMotorEx left;
//    private final DcMotorEx right;
//
//    private final VoltageSensor battery;
//
//    private final PIDFController pid = new PIDFController(kP, kI, kD, 0);
//    private final InterpLUT distToVelo;
//
//    private final Telemetry telemetry;
//
//    /* ---------------- Constructor ---------------- */
//    public Shooter(HardwareMap hw, Telemetry tele) {
//        this.telemetry = tele;
//
//        left = hw.get(DcMotorEx.class, "leftFlyWheel");
//        right = hw.get(DcMotorEx.class, "rightFlyWheel");
//        left.setDirection(DcMotorSimple.Direction.REVERSE);
//        right.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        battery = hw.voltageSensor.iterator().next();
//
//        pid.setTolerance(VELO_TOL);
//
//        distToVelo = buildLUT();
//    }
//    public void setMode(Mode mode) {
//        Shooter.mode = mode;
//    }
//    /* ---------------- LUT Setup ---------------- */
//    private InterpLUT buildLUT() {
//        InterpLUT lut = new InterpLUT();
//        lut.add(-100, 0);
//        lut.add(0, -970);
//        lut.add(2, -990);
//        lut.add(3, -1000);
//        lut.add(4, -1050);
//        lut.add(5, -1050);
//        lut.add(6, -1240);
//        lut.add(7, -1260);
//        lut.add(9, -1440);
//        lut.add(11, -1490);
//        lut.add(12, -1525);
//        lut.add(100, -1580);
//        lut.build();
//        return lut;
//    }
//
//    /* ---------------- Periodic Update ---------------- */
//    public void periodic(AprilTagDetection detections) {
//        switch (mode) {
//            case RAW:
//                applyRaw();
//                break;
//            case FIXED:
//                applyVelocity(targetVelocity);
//                break;
//            case DYNAMIC:
//                applyDynamic(detections);
//                break;
//        }
//        log();
//    }
//
//    /* ---------------- Public Control ---------------- */
//    public void setRaw(double p) {
//        mode = Mode.RAW;
//        targetPower = p;
//    }
//
//    public void setVelocity(double v) {
//        mode = Mode.FIXED;
//        targetVelocity = v;
//    }
//
//    public void setIdle() {
//        setVelocity(IDLE_VELO);
//    }
//
//    public boolean isAtTargetVelocity() {
//        return pid.atSetPoint();
//    }
//
//    /* ---------------- Control Modes ---------------- */
//    public double getVelocity() {
//        return right.getVelocity();
//    }
//
//    /** RAW MODE: direct motor power */
//    private void applyRaw() {
//        left.setPower(targetPower);
//        right.setPower(targetPower);
//    }
//
//    /** FIXED MODE: hold a target RPM using PIDF+FF */
//    private void applyVelocity(double target) {
//
//        if (tuning) {
//            pid.setPIDF(kP, kI, kD, 0);
//            pid.setTolerance(VELO_TOL);
//        }
//
//        double measured = right.getVelocity();
//        double ff = kV * target * voltageCompFactor();
//
//        double output = pid.calculate(measured, target) + ff;
//
//        left.setPower(output);
//        right.setPower(output);
//    }
//
//    /** DYNAMIC MODE: use Apriltag distance to determine velocity */
//    private void applyDynamic(AprilTagDetection detections) {
//        if (detections == null) {
//            // No tag → hold idle or power off
//            telemetry.addLine("No Tag");
package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;
@Config
public class Shooter {

    public enum Mode { RAW, FIXED, DYNAMIC }

    /* ---------------- Tunable Config Values ---------------- */
    public static double kP = 0.0001;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.00035;
    public static double VELO_TOL = 50;
    public static double IDLE_VELO = 100; // Match your LUT sign (negative)

    public static Mode mode = Mode.RAW;
    public static double shootVelocity = 700;
    public static double targetVelocity = shootVelocity;
    public static double targetPower = 0;
    public static double LUTKv = 1.33;

    private final DcMotorEx left, right;
    private final VoltageSensor battery;
    private final PIDController pid = new PIDController();
    private final InterpLUT distToVelo;
    private final Telemetry telemetry;

    public FtcDashboard dashboard;

    public Telemetry dashboardTelemetry;
    public Shooter(HardwareMap hw, Telemetry tele) {
        this.telemetry = tele;
        left = hw.get(DcMotorEx.class, "leftFlyWheel");
        right = hw.get(DcMotorEx.class, "rightFlyWheel");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        battery = hw.voltageSensor.iterator().next();
        distToVelo = buildLUT();
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

    }

    private InterpLUT buildLUT() {
        InterpLUT lut = new InterpLUT();
        lut.add(0, 400* LUTKv);
        lut.add(2, 450* LUTKv);
        lut.add(4, 530* LUTKv);
        lut.add(7, 570* LUTKv);
        lut.add(9, 750* LUTKv);
        lut.add(12, 880* LUTKv);
        lut.add(13, 900* LUTKv);
        lut.add(14, 980* LUTKv);
        lut.add(15, 1000* LUTKv);
        lut.build();
        return lut;
    }

//    public void periodic(AprilTagDetection detection) {
//        dashboardTelemetry.addData("Shooter Mode", mode);
//        switch (mode) {
//            case RAW:
//                left.setPower(targetPower);
//                right.setPower(targetPower);
//                break;
//            case FIXED:
//                applyVelocity(targetVelocity);
//                break;
//            case DYNAMIC:
//                if (detection != null) {
//                    double dist = Math.hypot(detection.ftcPose.x, detection.ftcPose.z);
//                    targetVelocity = distToVelo.get(Range.clip(dist, 0, 15));
//                    applyVelocity(targetVelocity);
//                    dashboardTelemetry.addLine("--- Vision Debug ---");
//                    dashboardTelemetry.addData("Raw Dist", dist); // You'll need to store this variable
//                    dashboardTelemetry.addData("LUT Output", distToVelo.get(dist));
//
//                } else {
//                    applyVelocity(IDLE_VELO);
//                    dashboardTelemetry.addLine("No Tag");
//                }
//                break;
//        }
//        log();
//    }

    public void periodic(AprilTagDetection detection) {
        dashboardTelemetry.addData("Shooter Mode", mode);

        switch (mode) {
            case RAW:
                left.setPower(targetPower);
                right.setPower(targetPower);
                break;

            case FIXED:
                applyVelocity(targetVelocity);
                break;

            case DYNAMIC:
                if (detection != null) {
                    // 1. Calculate distance in INCHES
                    double distInches = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
                    // 2. Convert to FEET (since your LUT is 0-15)
                    double distFeet = distInches / 12.0;

                    // 3. Update target velocity
                    targetVelocity = distToVelo.get(Range.clip(distFeet, 0, 15));

                    applyVelocity(targetVelocity);

                    // Debugging
                    dashboardTelemetry.addLine("--- Vision Debug ---");
                    dashboardTelemetry.addData("Dist (Inches)", distInches);
                    dashboardTelemetry.addData("Dist (Feet)", distFeet);
                    dashboardTelemetry.addData("Target RPM", targetVelocity);
                } else {
                    // 4. PREVENT SPIN DOWN:
                    // If we lose the tag, keep spinning at the last known targetVelocity
                    // instead of dropping to IDLE_VELO immediately.
                    applyVelocity(targetVelocity);

                    dashboardTelemetry.addLine("No Tag - Holding Last Velocity");
                }
                break;
        }
        log();
    }

    private void applyVelocity(double target) {
        double measured = right.getVelocity();
        dashboardTelemetry.addData("Measured", measured);
        dashboardTelemetry.addData("Target", target);

        // Feedforward handles the bulk of the power based on battery voltage
        double ff = kV * target * (13.0 / battery.getVoltage());

        // PID handles the correction
        double feedback = pid.calculate(measured, target);

        double power = ff + feedback;
        dashboardTelemetry.addData("Power", power);
        dashboardTelemetry.update();
        left.setPower(Range.clip(power, -1, 1));
        right.setPower(Range.clip(power, -1, 1));
    }

    public void setRaw(double p) { mode = Mode.RAW; targetPower = p; }
    public void setMode(Mode m) { mode = m; }
    public void setIdle() { mode = Mode.FIXED; targetVelocity = IDLE_VELO; }

    public boolean isAtTargetVelocity() {
        // Use Absolute value because target and measured are negative
        return Math.abs(targetVelocity - right.getVelocity()) < VELO_TOL;
    }

    public void setTargetVelocity(double v) {
        targetVelocity = v;
    }


    private void log() {
        telemetry.addData("Shooter Mode", mode);
        telemetry.addData("Target Velo", targetVelocity);
        telemetry.addData("Actual Velo", right.getVelocity());
        dashboardTelemetry.update();

    }

    /* --- Internal Classes --- */

    private class PIDController {
        double prevError = 0;
        double integral = 0;
        public double calculate(double measured, double target) {
            double error = target - measured;
            integral = Range.clip(integral + error, -2000, 2000);
            double derivative = error - prevError;
            prevError = error;
            return (kP * error) + (kI * integral) + (kD * derivative);
        }
    }

    public double getVelocity(){
        return left.getVelocity();
    }

    private static class InterpLUT {
        private final List<Double> xs = new ArrayList<>(), ys = new ArrayList<>();
        public void add(double x, double y) { xs.add(x); ys.add(y); }
        public void build() {} // Optional: add sorting logic here if needed
        public double get(double x) {
            if (x <= xs.get(0)) return ys.get(0);
            if (x >= xs.get(xs.size()-1)) return ys.get(ys.size()-1);
            for (int i = 0; i < xs.size() - 1; i++) {
                if (x <= xs.get(i+1)) {
                    double t = (x - xs.get(i)) / (xs.get(i+1) - xs.get(i));
                    return ys.get(i) + t * (ys.get(i+1) - ys.get(i));
                }
            }
            return ys.get(0);
        }
    }
}