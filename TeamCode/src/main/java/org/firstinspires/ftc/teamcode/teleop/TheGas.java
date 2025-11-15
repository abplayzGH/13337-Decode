package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.ballShooter;

@TeleOp(name = "The Gas", group = "Teleop")
public class TheGas extends LinearOpMode {

    double speedMult = 0.3;
    private ballShooter shooter = new ballShooter();
    private Intake intake = new Intake();



    @Override
    public void runOpMode() {
        // Initialize hardware
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        Servo feeder = hardwareMap.get(Servo.class, "feeder");
        feeder.setPosition(0);

        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.init(hardwareMap);
        intake.init(hardwareMap);


        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            // X = gamepad1 = wheel movement
            double leftX = gamepad1.left_stick_x; // Forward/Backward
            double leftY = gamepad1.left_stick_y;  // Left/Right
            double rightX = -gamepad1.right_stick_x; // Rotate


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


            // --- CORRECTED SHOOTER LOGIC ---
            // Determine if a shot is being requested by the operator
            boolean shotRequested = gamepad2.b;

            // The 'startLauncher()' method is redundant if 'updateState' is handled correctly.
            // We can let the state machine handle the SPIN_UP transition.
            if (!shotRequested) {
                shooter.stopLauncher(); // Resets state to IDLE and stops motors
            }


            // Always update the shooter's state machine.
            // Pass 'true' if the B button is pressed, which will trigger the launch sequence.
            shooter.updateState(shotRequested);
            // --- END OF CORRECTION ---


            if (gamepad2.left_bumper) {
                intake.runIntake();
            } else {
                intake.stopIntake();
            }

            double servo = 0.0;
            if(gamepad2.triangle){
                feeder.setPosition(90);
                sleep(750);
                feeder.setPosition(-90);
            }

//            if(gamepad2.square){
//                feeder.setPosition(-90);
//            }

            // Telemetry for debugging (optional)
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RR Power", rearRightPower);
            telemetry.update();

            //Update State
            shooter.updateState(false);

            telemetry.addData("State", shooter.getState());
            telemetry.update();
        }
    }

}