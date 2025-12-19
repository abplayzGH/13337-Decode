package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Turret {
    private DcMotorEx turretMotor;
    private static final int LEFT_LIMIT = -1000;
    private static final int RIGHT_LIMIT = 1000;
    private static final double kP = 0.02;
    private static final double kD = 0.001;
    private static final double MAX_AUTO_POWER = 0.7;

    private double lastError = 0.0;
    private double lastTime = 0.0;

    public void init(HardwareMap hardwareMap, String motorName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double limitPower(double power) {
        int currentPos = turretMotor.getCurrentPosition();
        if (currentPos <= LEFT_LIMIT && power < 0) return 0;
        if (currentPos >= RIGHT_LIMIT && power > 0) return 0;
        return power;
    }

    public void setManualPower(double power) {
        turretMotor.setPower(limitPower(power));
    }

    public double updateTurretTracking(AprilTagDetection detection, double currentTime) {
        if (detection != null) {
            double currentBearing = detection.ftcPose.bearing;
            double dt = currentTime - lastTime;
            if (dt < 0.005) return turretMotor.getPower();

            double error = 0.0 - currentBearing;
            double derivative = (error - lastError) / dt;
            double correction = (error * kP) + (derivative * kD);

            lastError = error;
            lastTime = currentTime;

            double drivePower = Range.clip(correction, -MAX_AUTO_POWER, MAX_AUTO_POWER);
            double limitedPower = limitPower(drivePower);
            turretMotor.setPower(limitedPower);
            return limitedPower;
        } else {
            turretMotor.setPower(0.0);
            return 0.0;
        }
    }

    public int getPosition() { return turretMotor.getCurrentPosition(); }
}