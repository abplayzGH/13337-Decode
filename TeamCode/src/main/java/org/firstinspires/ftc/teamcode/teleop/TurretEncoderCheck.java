package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Turret Encoder Check", group = "Diagnostics")
public class TurretEncoderCheck extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMap hw = hardwareMap;
        DcMotorEx turret = null;
        try {
            turret = hw.get(DcMotorEx.class, "turret_motor");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Could not find 'turret_motor' in hardware map");
            telemetry.addLine("Check robot configuration name and type");
            telemetry.update();
            waitForStart();
            return;
        }

        // Reset and enable encoder mode
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready. Use D-pad Up to spin forward, Down to spin reverse, A to stop.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.0;
            if (gamepad1.dpad_up) power = 0.25;
            else if (gamepad1.dpad_down) power = -0.25;
            else if (gamepad1.a) power = 0.0;

            turret.setPower(power);

            telemetry.addData("Power", String.format("%.2f", power));
            telemetry.addData("Position", turret.getCurrentPosition());
            telemetry.addData("Mode", turret.getMode());
            telemetry.addData("ZeroPowerBehavior", turret.getZeroPowerBehavior());
            telemetry.update();

            sleep(100);
        }
    }
}

