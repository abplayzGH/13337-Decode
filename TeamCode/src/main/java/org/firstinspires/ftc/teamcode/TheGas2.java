package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.ballShooter;

@TeleOp(name = "TheGas2")
public class TheGas2 extends LinearOpMode {
    ballShooter ballShooter = new ballShooter();
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    public DcMotorEx leftFlyWheel;
    public DcMotorEx rightFlyWheel;
    public DcMotorEx conveyorBelt;
    //private DcMotorEx extend;
    // private Servo angle;
    // private Servo claw;
    // double anglePos = 0.48;
    // double angleMult = -0.0033;
    // double extendMult = -0.6;
    // double clawPower = 0.0;
    // double clawPos = 0.3;
    // double clawMult = 0.0005;
    double speedMult = 0.3;
    boolean inAuto = false;


    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightFlyWheel");
        conveyorBelt = hardwareMap.get(DcMotorEx.class, "conveyorBelt");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFlyWheel.setPower(1.0);
        rightFlyWheel.setPower(1.0);
        conveyorBelt.setPower(0.5);
/*
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        angle = hardwareMap.get(Servo.class, "angle");
        // angle.setPosition(anglePos);
        anglePos = angle.getPosition();

        claw = hardwareMap.get(Servo.class, "claw");
*/
        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Instance of autonomous class
        // AutoMethods auto = new AutoMethods(leftFront, leftRear, rightFront, rightRear, extend, angle, claw);
        // auto.extendSpeed = 0.5;

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            // X = gamepad1 = wheel movement
            double leftX = gamepad1.left_stick_x; // Forward/Backward
            double leftY = gamepad1.left_stick_y;  // Left/Right
            double rightX = -gamepad1.right_stick_x; // Rotate
            double firePower = 1.0;
/*          double firePower = 1.0
            double extendPower = gamepad2.left_stick_y; // extend
            double anglePower = gamepad2.right_stick_y * angleMult; // wrist
            if (anglePower < 0.0025 && anglePower > -0.0025) {
                anglePower = 0.0;
            } else if (anglePower > 0.0025) {
                anglePower = 0.0025;
            } else if (anglePower < -0.0025) {
                anglePower = -0.0025;
            }

            if (gamepad2.left_bumper) // claw
                clawPower = 1;
            else if (gamepad2.right_bumper)
                clawPower = -1;
            else
                clawPower = 0;
*/
            // Calculate motor power
            // double frontLeftPower = leftY - leftX + rightX;
            // double rearLeftPower = leftY + leftX + rightX;
            // double frontRightPower = leftY + leftX - rightX;
            // double rearRightPower = leftY - leftX - rightX;

            //Calculate motor power
            double frontLeftPower = leftY - leftX + rightX;
            double rearLeftPower = leftY + leftX + rightX;
            double frontRightPower = leftY + leftX - rightX;
            double rearRightPower = leftY - leftX - rightX;


            // Normalize the power values if any exceed 1.0
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(rearLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                rearLeftPower /= maxPower;
                frontRightPower /= maxPower;
                rearRightPower /= maxPower;
            }

            speedMult = 0.3 + (0.7*gamepad1.right_trigger);


            // Set wheel power
            leftFront.setPower(frontLeftPower * speedMult);
            leftBack.setPower(rearLeftPower * speedMult);
            rightFront.setPower(frontRightPower * speedMult);
            rightBack.setPower(rearRightPower * speedMult);

            // extend
            // extend.setPower((extendPower * extendMult) + extend.getCurrentPosition()*0.000002);

            // Pick up specimen
//            if (gamepad2.cross){
//                anglePos = 0.5;
//                clawPos = 0.28;
//            }

            // Place specimen if not using auto
//            if (gamepad1.square){
//                ballShooter.shoot(firePower);
//            }
/*
            // place specimen auto
            if (gamepad1.triangle){
                inAuto = true;
                extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // auto.closeClaw();
                angle.setPosition(0.65);
                sleep((long)500.0);
                auto.extendUp(0.6);

                leftFront.setPower(0.3);
                rightFront.setPower(0.3);
                leftRear.setPower(0.3);
                rightRear.setPower(0.3);
                sleep((long)1000.0);
                leftFront.setPower(0.0);
                rightFront.setPower(0.0);
                leftRear.setPower(0.0);
                rightRear.setPower(0.0);

                auto.backward(8.5);
                auto.extendDown(0.1);
                auto.openClaw();
                sleep((long)800.0);
                auto.backward(4.0);
                auto.wristAngle(0.7);
                extend.setPower(-0.7);
                sleep((long)1000.0);
                extend.setPower(0.0);
                clawPos = claw.getPosition();
                anglePos = angle.getPosition();
                inAuto = false;
            }

            // pick up specimen off wall
            if (gamepad1.square){
                inAuto = true;
                auto.wristAngle(0.47);
                auto.openClaw();
                sleep((long)1000.0);

                leftFront.setPower(0.2);
                rightFront.setPower(0.2);
                leftRear.setPower(0.2);
                rightRear.setPower(0.2);
                sleep((long)700.0);
                leftFront.setPower(0.0);
                rightFront.setPower(0.0);
                leftRear.setPower(0.0);
                rightRear.setPower(0.0);

                auto.closeClaw();
                auto.extendUp(0.14);
                auto.backward(5.0);
                auto.wristAngle(0.7);
                extend.setPower(-0.7);
                sleep((long)1000.0);
                extend.setPower(0.0);
                clawPos = claw.getPosition();
                anglePos = angle.getPosition();
                inAuto = false;
            }

            if (gamepad1.circle) {
                sleep((long)3000.0);
            }

            if (!inAuto){
                // wrist
                anglePos += anglePower;
                if (anglePos < 0.32)
                    anglePos = 0.33;
                else if (anglePos > 0.68)
                    anglePos = 0.68;
                angle.setPosition(anglePos);

                // claw
                clawPos += clawPower * clawMult;
                if (clawPos < 0.25)
                    clawPos = 0.25;
                else if (clawPos > 0.325)
                    clawPos = 0.325;
                claw.setPosition(clawPos);
            }
            */

            // Telemetry for debugging (optional)
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RR Power", rearRightPower);
            // telemetry.addData("Extend Power", extendPower);
            // telemetry.addData("Wrist Position", angle.getPosition());
            // telemetry.addData("Claw Position", claw.getPosition());
            telemetry.update();
        }
    }

}