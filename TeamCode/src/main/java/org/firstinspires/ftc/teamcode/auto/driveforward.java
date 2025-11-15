package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class driveforward extends LinearOpMode {
    private MecanumDrive drive;
    ElapsedTime timer = new ElapsedTime();


    public void runOpMode() throws InterruptedException {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1.0, 0), 0.0));
        timer.seconds();
        if (timer.seconds() > 2) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0.0));

        }

    }
}
