package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class ballShooterv2 {

    public DcMotorEx leftFlyWheel;
    public DcMotorEx rightFlyWheel;
    public DcMotorEx conveyorBelt;

    public Servo coolServo;
    public static final double TUNING_CONSTANT = 0.4; // This is a starting value, you MUST tune it.

    final double FEED_TIME = 0.20; //The feeder servos run this long when a shot is requested.

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    final double TIME_BETWEEN_SHOTS = 2;



    public void init(HardwareMap hardwareMap) {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightFlyWheel");
        conveyorBelt = hardwareMap.get(DcMotorEx.class, "conveyorBelt");
        coolServo = hardwareMap.get(Servo.class, "coolServo");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);

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
        double calculatedPowerL = TUNING_CONSTANT / Math.sin(angleRadians);
        double calculatedPowerR = TUNING_CONSTANT / Math.cos(angleRadians);

        // 4. Clip the final power to the acceptable range for DC motors [0.0, 1.0].
        // We only want positive power for launching.
        double finalPowerL = Range.clip(calculatedPowerL, 0.0, 1.0);
        double finalPowerR = Range.clip(calculatedPowerR, 0.0, 1.0);

        // 5. Set the calculated power to both flywheel motors.
        leftFlyWheel.setPower(finalPowerL);
        rightFlyWheel.setPower(finalPowerR);

        try {
            Thread.sleep(500); // 1000 ms = 1 second
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        conveyorBelt.setPower(1.0);

    }

    public void stop() {
        try {
            leftFlyWheel.setPower(0);
            rightFlyWheel.setPower(0);
            conveyorBelt.setPower(0);
        } catch (Exception ignored){
        }
    }


}
