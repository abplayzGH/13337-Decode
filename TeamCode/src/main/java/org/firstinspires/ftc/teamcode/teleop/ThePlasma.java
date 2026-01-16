package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class ThePlasma extends LinearOpMode {

    public Robot robot = new Robot(hardwareMap, telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeIsActive()) {
            robot.latch.setPosition(0.1);
        }
    }


}