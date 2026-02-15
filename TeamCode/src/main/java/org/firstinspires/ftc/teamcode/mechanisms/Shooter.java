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

import java.util.ArrayList;
import java.util.List;
@Config
public class Shooter {

    public enum Mode { RAW, FIXED, DYNAMIC }

    /* ---------------- Tunable Config Values ---------------- */
    public static double kP = 0.0001;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.000444;
    public static int VELO_TOL = 20;
    public static double IDLE_VELO = 100; // Match your LUT sign (negative)

    public static Mode mode = Mode.RAW;
    public static double shootVelocity = 700;
    public static double targetVelocity = shootVelocity;
    public static double targetPower = 0;
    public static double LUTKv = 1;

    private final DcMotorEx left, right;
    private final VoltageSensor battery;
    private final PIDController pid = new PIDController();
    private final InterpLUT distToVelo;
    public FtcDashboard dashboard;
    public Telemetry dashboardTelemetry;

//    private Robot robot;

    public Shooter(HardwareMap hw, Telemetry telemetry) {
//        this.robot = robotInstance; // Store the reference to the robot that owns this shooter

        left = hw.get(DcMotorEx.class, "leftFlyWheel");
        right = hw.get(DcMotorEx.class, "rightFlyWheel");
        // ... rest of your hardware setup ...

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        right.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        left.setTargetPositionTolerance(VELO_TOL);
        right.setTargetPositionTolerance(VELO_TOL);

        battery = hw.voltageSensor.iterator().next();
        distToVelo = buildLUT();

        dashboard = FtcDashboard.getInstance();
        // Use provided telemetry for local dashboardTelemetry to avoid unused parameter warning
        dashboardTelemetry = telemetry != null ? telemetry : dashboard.getTelemetry();
    }
    //Units in inches and RPM
    private InterpLUT buildLUT() { //TODO Tune this
        InterpLUT lut = new InterpLUT();
        lut.add(-100, 0);
        lut.add(0, 970);
        lut.add(78.7402, 990* LUTKv);
        lut.add(118.11, 1000* LUTKv);
        lut.add(157.48, 1050* LUTKv);
        lut.add(196.85, 1050* LUTKv);
        lut.add(236.22, 1240* LUTKv);
        lut.add(275.591, 1260* LUTKv);
        lut.add(354.331, 1440* LUTKv);
        lut.add(433.071, 1490* LUTKv);
        lut.add(472.441, 1525* LUTKv);
        lut.add(3937.01, 1580* LUTKv);
        lut.build();
        return lut;
    }

    /** DYNAMIC MODE: use Limelight distance (inches) to determine velocity */
    public void periodic(Double dist) {
        switch (mode) {
            case RAW:
                left.setPower(targetPower);
                right.setPower(targetPower);
                break;

            case FIXED:
                applyVelocity(targetVelocity);
                break;

            case DYNAMIC:
                if (dist != null && dist > 0) {
                    // You must re-tune your buildLUT() to use 'ta' values as the 'x'
                    targetVelocity = distToVelo.get(dist);
                    applyVelocity(targetVelocity);

                    dashboardTelemetry.addData("Limelight TA", dist);
                    dashboardTelemetry.addData("Target RPM", targetVelocity);
                } else {
                    // If tag lost, maintain last known velocity to keep flywheels spinning
                    applyVelocity(targetVelocity);
                }
                break;
        }
    }
    @Deprecated
    private void applyVelocityOld(double target) {
        double measured = right.getVelocity();
        dashboardTelemetry.addData("Measured", measured);
        dashboardTelemetry.addData("Target", target);

        // Feedforward handles the bulk of the power based on battery voltage
        double ff = kF * target * (13.0 / battery.getVoltage());

        // PID handles the correction
        double feedback = pid.calculate(measured, target);

        double power = ff + feedback;
        left.setPower(Range.clip(power, -1, 1));
        right.setPower(Range.clip(power, -1, 1));
    }

    private void applyVelocity(double target) {
        left.setVelocity(target);
        right.setVelocity(target);

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


//    private void log() {
//        telemetry.addData("Shooter Mode", mode);
//        telemetry.addData("Target Velo", targetVelocity);
//        telemetry.addData("Actual Velo", right.getVelocity());
//        robot.dashboardTelemetry.update();
//
//    }

    /* --- Internal Classes --- */
    @Deprecated
    private static class PIDController {
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
        return right.getVelocity(); // Idiot use right motor lol
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

