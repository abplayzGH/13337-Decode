package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumDrive {

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private IMU imu;
    public void init(HardwareMap hardwareMap){

//        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
//        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
//        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront = hardwareMap.get(DcMotorEx.class, "motor");
        leftRear = hardwareMap.get(DcMotorEx.class, "motor");
        rightFront = hardwareMap.get(DcMotorEx.class, "motor");
        rightRear = hardwareMap.get(DcMotorEx.class, "motor");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate){
        double leftFrontPower = forward + strafe + rotate;
        double leftRearPower =  forward - strafe + rotate;
        double rightFrontPower = forward - strafe - rotate;
        double rightRearPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));

        leftFront.setPower(maxSpeed * (leftFrontPower / maxPower));
        leftRear.setPower(maxSpeed * (leftRearPower / maxPower));
        rightFront.setPower(maxSpeed * (rightFrontPower / maxPower));
        rightRear.setPower(maxSpeed * (rightRearPower / maxPower));

    }

    public void driveFieldRelative(double forward, double strafe, double rotate){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

}
