package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Autonomous(name="driveforward")
public class driveforward extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {

        // Map motors (names must match the Robot Config)
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Reverse left side for mecanum drive
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Drive forward
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        leftBack.setPower(-0.5);
        rightBack.setPower(-0.5);

        sleep(2000); // 2 seconds

        // Stop motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
