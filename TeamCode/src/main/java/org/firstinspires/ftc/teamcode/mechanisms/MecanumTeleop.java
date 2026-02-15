package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumTeleop {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    double speedMult = 0.3;
    public void Init(HardwareMap hw){
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
    }

    public void Drive(double leftX, double leftY, double rightX, double speed){
        double y = leftY;
        double x = -leftX;
        double rx = rightX;

        // Calculate motor power (Standard Mecanum Kinematics)
        double frontLeftPower  = y + x + rx;
        double rearLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double rearRightPower  = y + x - rx;

        // Normalize power
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(rearLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            rearLeftPower /= maxPower;
            frontRightPower /= maxPower;
            rearRightPower /= maxPower;
        }

        // Set wheel power
        leftFront.setPower(frontLeftPower * speed);
        leftBack.setPower(rearLeftPower * speed);
        rightFront.setPower(frontRightPower * speed);
        rightBack.setPower(rearRightPower * speed);
    }


}
