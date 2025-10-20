package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class ballShooter {

    public DcMotorEx leftFlyWheel;
    public DcMotorEx rightFlyWheel;
    public DcMotorEx conveyorBelt;
    public static final double TUNING_CONSTANT = 0.6; // This is a starting value, you MUST tune it.


    public void init(HardwareMap hardwareMap) {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightFlyWheel");
        conveyorBelt = hardwareMap.get(DcMotorEx.class, "conveyorBelt");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set the motors to run without encoders, as we are controlling power directly.
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the behavior when motor power is zero. BRAKE can help prevent backspin.
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void shoot(double power) {
        leftFlyWheel.setPower(power);
        rightFlyWheel.setPower(power);

        try {
            Thread.sleep(500); // 1000 ms = 1 second
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        conveyorBelt.setPower(1.0);
    }

    public void moveConveyorBelt(double power){
        conveyorBelt.setPower(power);
    }

    public void setLaunchPowerFromAngle(double angleDegrees) {
        // 1. Constrain the angle to a safe range (e.g., 0 to 85 degrees)
        // This prevents Math errors from Math.cos(90) which is zero.
        double safeAngleDegrees = Range.clip(angleDegrees, 0.0, 85.0);

        // 2. Convert the angle from degrees to radians for use in Java's Math functions.
        double angleRadians = Math.toRadians(safeAngleDegrees);

        // 3. Calculate the power based on a simplified physics model.
        // Power is inversely proportional to the cosine of the angle.
        // This means that as the launcher aims higher (a steeper angle), the power increases.
        double calculatedPower = TUNING_CONSTANT / Math.cos(angleRadians);

        // 4. Clip the final power to the acceptable range for DC motors [0.0, 1.0].
        // We only want positive power for launching.
        double finalPower = Range.clip(calculatedPower, 0.0, 1.0);

        // 5. Set the calculated power to both flywheel motors.
        leftFlyWheel.setPower(finalPower);
        rightFlyWheel.setPower(finalPower);

        try {
            Thread.sleep(500); // 1000 ms = 1 second
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        conveyorBelt.setPower(1.0);

    }

    public void stop() {
        leftFlyWheel.setPower(0);
        rightFlyWheel.setPower(0);
        conveyorBelt.setPower(0);
    }


}
