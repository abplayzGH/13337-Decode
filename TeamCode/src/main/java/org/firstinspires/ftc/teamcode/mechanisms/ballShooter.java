package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class ballShooter {

    public DcMotorEx leftFlyWheel;
    public DcMotorEx rightFlyWheel;
    public DcMotorEx conveyorBelt;
    public Servo feeder;

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * Here we create three timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */

    // Declare OpMode members.

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step of a state
     * machine is creating an enum that captures the different "states" that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
     * and only run the bits of code we need to at different times. This state machine is called the
     * "LaunchState." It reflects the current condition of the shooter motor when we request a shot.
     * It starts at IDLE. When a shot is requested from the user, it'll move into PREPARE then LAUNCH.
     * We can use higher level code to cycle through these states, but this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits."
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }


    /*
     * Here we create the instance of LaunchState that we use in code. This creates a unique object
     * which can store the current condition of the shooter. In other applications, you may have
     * multiple copies of the same enum which have different names. Here we just have one.
     */
    private LaunchState launchState;


    public void init(HardwareMap hardwareMap) {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightFlyWheel");
        conveyorBelt = hardwareMap.get(DcMotorEx.class, "conveyorBelt");
        feeder = hardwareMap.get(Servo.class, "feeder");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the motors to run without encoders, as we are controlling power directly.
        leftFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the behavior when motor power is zero. BRAKE can help prevent backspin.
        leftFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchState = LaunchState.IDLE;

        feeder.setPosition(0);
        stopLauncher();


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

    public void moveConveyorBelt(double power) {
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
        double calculatedPowerL = Math.sin(angleRadians);
        double calculatedPowerR = Math.cos(angleRadians);

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
        } catch (Exception ignored) {
        }
    }

    public void updateState(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                leftFlyWheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
                rightFlyWheel.setVelocity(LAUNCHER_TARGET_VELOCITY);
                conveyorBelt.setPower(1);
                if (leftFlyWheel.getVelocity() > LAUNCHER_MIN_VELOCITY && rightFlyWheel.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feeder.setPosition(1);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    feeder.setPosition(0);
                    conveyorBelt.setPower(1);

                }
                break;
        }

    }

    public void startLauncher() {
        if (launchState == LaunchState.IDLE) {
            //Transition State
            launchState = LaunchState.SPIN_UP;
        }
    }

    public void stopLauncher() {
        leftFlyWheel.setPower(0);
        rightFlyWheel.setPower(0);
        conveyorBelt.setPower(0);
        launchState = LaunchState.IDLE;
    }

    public String getState() {
        return launchState.toString();
    }

}

