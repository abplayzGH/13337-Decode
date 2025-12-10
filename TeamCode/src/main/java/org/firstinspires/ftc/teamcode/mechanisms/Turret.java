package org.firstinspires.ftc.teamcode.mechanisms; // Suggested package structure

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Subsystem responsible for controlling the turret motor and implementing
 * AprilTag-based Proportional-Derivative (PD) tracking.
 */
public class Turret {

    private DcMotorEx turretMotor;
    private long startTimeNs;

    // --- PD CONTROL CONSTANTS ---
    private static final double kP = 0.02;
    private static final double kD = 0.001;
    private static final double MAX_POWER = 0.7;
    private static final double MIN_POWER = 0.1;

    // --- PD CONTROL STATE ---
    private double lastError = 0.0;
    private double lastTime = 0.0;
    private final double targetBearing = 0.0; // Target is always 0.0 (centered)

    public void TurretSubsystem(HardwareMap hardwareMap, String motorName) {
        // Map the turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Capture the start time for derivative calculation
        startTimeNs = System.nanoTime();
    }

    /**
     * Attempts to track a target AprilTag. If a detection is provided,
     * it calculates and applies motor power. Otherwise, it stops the motor.
     * * @param detection The AprilTagDetection object (null if no tag is visible).
     * @param currentTime The current time in seconds, provided by the OpMode's runtime.
     * @return The calculated motor power for telemetry/debugging.
     */
    public double updateTurretTracking(AprilTagDetection detection, double currentTime) {
        if (detection != null) {
            double currentBearing = detection.ftcPose.bearing;

            // Calculate time delta for stable derivative
            double dt = currentTime - lastTime;
            if (dt < 0.005) { // Protect against divide by zero or very small delta
                return turretMotor.getPower();
            }

            // Calculate the error
            double error = targetBearing - currentBearing;

            // Calculate the derivative: Rate of change of the error
            double derivative = (error - lastError) / dt;

            // Calculate the power using the PD formula
            double correction = (error * kP) + (derivative * kD);

            // Update state variables for the next iteration
            lastError = error;
            lastTime = currentTime;

            // Apply power limits and minimum power (Deadzone)
            double drivePower = correction;
            if (Math.abs(drivePower) > MAX_POWER) {
                drivePower = Math.copySign(MAX_POWER, drivePower);
            } else if (Math.abs(drivePower) > 0.01 && Math.abs(drivePower) < MIN_POWER) {
                // Apply minimum power to overcome friction
                drivePower = Math.copySign(MIN_POWER, drivePower);
            } else if (Math.abs(drivePower) <= 0.01) {
                drivePower = 0.0; // Stop motor if error is effectively zero
            }

            turretMotor.setPower(drivePower);
            return drivePower;

        } else {
            // Stop motor if tag is not visible
            turretMotor.setPower(0.0);
            lastTime = currentTime;
            return 0.0;
        }
    }

    /**
     * Emergency method to stop the turret movement.
     */
    public void stop() {
        turretMotor.setPower(0.0);
    }
}