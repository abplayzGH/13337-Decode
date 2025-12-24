package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrive {

    /* ---------------- Hardware ---------------- */
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private IMU imu;

    /* ---------------- Heading Hold ---------------- */
    private double targetHeading = 0;

    /* ---------------- Tunables ---------------- */
    private static final double HEADING_KP = 1.5;
    private static final double TURN_CLAMP = 0.4;
    private static final double STICK_DEADZONE = 0.05;

    /* ---------------- Init ---------------- */
    public void init(HardwareMap hw) {

        leftFront  = hw.get(DcMotorEx.class, "leftFront");
        leftBack   = hw.get(DcMotorEx.class, "leftBack");
        rightFront = hw.get(DcMotorEx.class, "rightFront");
        rightBack  = hw.get(DcMotorEx.class, "rightBack");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        DcMotorEx[] motors = {leftFront, leftBack, rightFront, rightBack};
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //TODO Adjust based on your hub orientation
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        targetHeading = getHeading();
    }

    /* ---------------- Main Drive Method ---------------- */
    public void drive(double x, double y, double rx, double speed) {

        // Deadzones
        x  = Math.abs(x)  > STICK_DEADZONE ? x  : 0;
        y  = Math.abs(y)  > STICK_DEADZONE ? y  : 0;
        rx = Math.abs(rx) > STICK_DEADZONE ? rx : 0;

        double heading = getHeading();

        boolean driverTurning = Math.abs(rx) > 0.05;
        if (driverTurning) {
            targetHeading = heading;
        }

        /* -------- Field-Centric Transform -------- */
        double cosA = Math.cos(-heading);
        double sinA = Math.sin(-heading);

        double fieldX = x * cosA - y * sinA;
        double fieldY = x * sinA + y * cosA;

        /* -------- Heading Hold -------- */
        double turn;
        if (driverTurning) {
            turn = rx;
        } else {
            double error = targetHeading - heading;
            turn = Range.clip(error * HEADING_KP, -TURN_CLAMP, TURN_CLAMP);
        }

        /* -------- Mecanum Math -------- */
        double fl = fieldY - fieldX + turn;
        double bl = fieldY + fieldX + turn;
        double fr = fieldY + fieldX - turn;
        double br = fieldY - fieldX - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br)));
        if (max > 1.0) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }

        leftFront.setPower(fl * speed);
        leftBack.setPower(bl * speed);
        rightFront.setPower(fr * speed);
        rightBack.setPower(br * speed);
    }

    /* ---------------- Utilities ---------------- */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        targetHeading = getHeading();
    }
}
