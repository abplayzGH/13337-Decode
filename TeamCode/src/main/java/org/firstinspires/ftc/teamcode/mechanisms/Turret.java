package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {

    private DcMotorEx turretMotor;
    private final ElapsedTime timer = new ElapsedTime();

    // ---- Control Constants ----
    public static double kP = 0.025;
    public static double kD = 0.007;
    public static double kF = 0.007;      // Feedforward (helps overcome friction)
    public static double MAX_AUTO_POWER = 0.6;
    public static double DEADZONE_DEG = 0.5;
    public static int LEFT_LIMIT = -1000;
    public static int RIGHT_LIMIT = 1000;


    // ---- State ----
    private double lastError = 0.0;
    private double lastTime = 0.0;

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
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
            turretMotor.setPower(0);
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

        turretMotor.setPower(power);

        lastError = error;
        lastTime = currentTime;

    }

    private void resetController(double time) {
        lastError = 0;
        lastTime = time;
    }

    public double getPosition() {
        return turretMotor.getCurrentPosition();
    }
}
