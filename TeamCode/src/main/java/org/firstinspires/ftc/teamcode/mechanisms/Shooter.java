package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;
@Config
public class Shooter {

    public enum Mode { RAW, FIXED, DYNAMIC }

    /* ---------------- Tunable Config Values ---------------- */
    public static double kP = 0.003;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.000551;
    public static double VELO_TOL = 20;
    public static double IDLE_VELO = 600; // Match your LUT sign (negative)

    public static Mode mode = Mode.RAW;
//    public static double shootVelocity = Robot.SHOOTER_READY_VELOCITY;
    public static double targetVelocity = Robot.SHOOTER_READY_VELOCITY;
    public static double targetPower = 0;
    public static double LUTKv = 1;

    private final DcMotorEx left, right;
    private final VoltageSensor battery;
    private final PIDController pid = new PIDController();
    private final InterpLUT distToVelo;

    private final Robot robot;

    public Shooter(HardwareMap hw, Telemetry telemetry) {
//        this.robot = robotInstance; // Store the reference to the robot that owns this shooter
        robot = Robot.get();

        left = hw.get(DcMotorEx.class, "leftFlyWheel");
        right = hw.get(DcMotorEx.class, "rightFlyWheel");
        // ... rest of your hardware setup ...

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        battery = hw.voltageSensor.iterator().next();
        distToVelo = buildLUT();

        // NO MORE Robot.get().Init() here!
    }

    private InterpLUT buildLUT() {
        InterpLUT lut = new InterpLUT();
        // Add from SMALLEST X to LARGEST X
        // (-80 is the smallest number, -13.5 is the largest)
        lut.add(-80.0, 1510 * LUTKv);
        lut.add(-75.0, 1460 * LUTKv);
        lut.add(-38.0, 1170 * LUTKv);
        lut.add(-20.5, 1150 * LUTKv);
        lut.add(-13.5, 900 * LUTKv);

        lut.build();
        return lut;
    }

    /** DYNAMIC MODE: use Limelight Target Area (ta) to determine velocity */
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
                if (dist != null) {
                    // targetArea is the percentage of the image (0-100)
                    // You must re-tune your buildLUT() to use 'ta' values as the 'x'
                    targetVelocity = distToVelo.get(dist);
                    applyVelocity(targetVelocity);
                    robot.flightRecorder.addData("Target dist", dist);

                    robot.flightRecorder.addData("Target RPM", targetVelocity);
                } else {
                    // If tag lost, maintain last known velocity to keep flywheels spinning
                    applyVelocity(targetVelocity);
                }
                break;
        }
    }

    private void applyVelocity(double target) {
        double measured = right.getVelocity();
        robot.flightRecorder.addData("Measured", measured);
        robot.flightRecorder.addData("Target", target);

        // Feedforward handles the bulk of the power based on battery voltage
        double ff = kV * target * (13.0 / battery.getVoltage());

        // PID handles the correction
        double feedback = pid.calculate(measured, target);

        double power = ff + feedback;
//        robot.dashboardTelemetry.addData("Power", power);
//        robot.dashboardTelemetry.update();
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


//    private void log() {
//        telemetry.addData("Shooter Mode", mode);
//        telemetry.addData("Target Velo", targetVelocity);
//        telemetry.addData("Actual Velo", right.getVelocity());
//        robot.dashboardTelemetry.update();
//
//    }

    /* --- Internal Classes --- */

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
        return left.getVelocity();
    }

//    private static class InterpLUT {
//        private final List<Double> xs = new ArrayList<>(), ys = new ArrayList<>();
//        public void add(double x, double y) { xs.add(x); ys.add(y); }
//        public void build() {} // Optional: add sorting logic here if needed
//        public double get(double x) {
//            if (x <= xs.get(0)) return ys.get(0);
//            if (x >= xs.get(xs.size()-1)) return ys.get(ys.size()-1);
//            for (int i = 0; i < xs.size() - 1; i++) {
//                if (x <= xs.get(i+1)) {
//                    double t = (x - xs.get(i)) / (xs.get(i+1) - xs.get(i));
//                    return ys.get(i) + t * (ys.get(i+1) - ys.get(i));
//                }
//            }
//            return ys.get(0);
//        }
//    }

    public class InterpLUT {
        ArrayList<Double> xs = new ArrayList<>();
        ArrayList<Double> ys = new ArrayList<>();

        public void add(double x, double y) {
            xs.add(x);
            ys.add(y);
        }    public void build() {
            // No changes needed here, just ensure they are added in ascending order (-80, -75...)
        }

        public double get(double x) {
            if (xs.isEmpty()) return 0;

            // 1. Handle Out of Bounds (Closest)
            if (x <= xs.get(0)) return ys.get(0);

            // 2. Handle Out of Bounds (Furthest)
            if (x >= xs.get(xs.size() - 1)) return ys.get(ys.size() - 1);

            // 3. Interpolate
            for (int i = 0; i < xs.size() - 1; i++) {
                double x0 = xs.get(i);
                double x1 = xs.get(i + 1);

                if (x >= x0 && x <= x1) {
                    double y0 = ys.get(i);
                    double y1 = ys.get(i + 1);

                    // Linear interpolation formula: y = y0 + (x - x0) * ((y1 - y0) / (x1 - x0))
                    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
                }
            }
            return ys.get(ys.size() - 1);
        }
    }
}