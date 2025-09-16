package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagVision;
import org.firstinspires.ftc.teamcode.mechanisms.MotorTest;

@Autonomous(name = "AutoTest", group = "Linear Opmode")
public class AutoTest extends LinearOpMode {

    MotorTest motor = new MotorTest();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("g", "dfs");
//
//        AprilTagVision vision = new AprilTagVision(hardwareMap);
//        motor.init(hardwareMap);
//
//
//        telemetry.addLine("Initialized. Waiting for start...");
//        telemetry.update();
//
//        waitForStart();
//        motor.setMotorSpeed(0.5);
//
//        while (!isStopRequested() && opModeIsActive()) {
//            // Example: pick action based on detected AprilTag
//            if (!vision.getDetections().isEmpty()) {
//                int id = vision.getDetections().get(0).id;
//                telemetry.addData("First tag ID", id);
//
//                if (id == 21) {
//                    telemetry.addLine("GPP");
//                } else if (id == 22) {
//                    telemetry.addLine("PGP");
//                } else if (id == 23) {
//                    telemetry.addLine("PPG");
//                }
//            } else {
//                telemetry.addLine("No AprilTags detected - default action");
//            }
//
//            telemetry.update();
//        }
    }
}
