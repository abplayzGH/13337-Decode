package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTest;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "Field Orientated", group = "Teleop")
public class MecanumFieldOrientatedOpMode extends OpMode {
    MecanumDriveTest drive = new MecanumDriveTest();
    double forward, strafe, rotate;

    @Override
    public void init(){

        drive.init(hardwareMap);
        drive.resetFieldCentric();
        telemetry.addData("Alliance", Robot.alliance);

        if (Robot.alliance == Robot.Alliance.RED) {
            drive.odometryUpdate(Math.PI);
        } else if (Robot.alliance == Robot.Alliance.BLUE){
            drive.odometryUpdate(0);
        }
    }

    @Override
    public void loop(){
        forward = -gamepad1.left_stick_y; // Negate this
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFieldCentric(forward, strafe, rotate, 1);
        telemetry.addData("X", drive.getPosX());
        telemetry.addData("Y", drive.getPosY());
        telemetry.addData("Heading", drive.getHeading());
        telemetry.update();
    }
}
