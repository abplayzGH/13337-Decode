package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.ballShooter;

@TeleOp(name = "The Plasma", group = "Teleop")
public class ThePlasma extends OpMode {
    private MecanumDrive drive;

    private double translationalScale = 1.0;
    private double rotationalScale = 1.0;

    private static final double JOYSTICK_DEADZONE = 0.05;

    private ballShooter shooter = new ballShooter();

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        shooter.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Raw joystick inputs
        double rawFwd = -gamepad1.left_stick_y;
        double rawStr =  gamepad1.left_stick_x;
        double rawRot =  gamepad1.right_stick_x;

        // Deadzone & scale
        double fwd = applyDeadzone(rawFwd, JOYSTICK_DEADZONE) * translationalScale;
        double str = applyDeadzone(rawStr, JOYSTICK_DEADZONE) * translationalScale;
        double rot = applyDeadzone(rawRot, JOYSTICK_DEADZONE) * rotationalScale;

        // Get current robot heading (in radians)
        // You can get it from your IMU or localizer; here’s one way assuming your MecanumDrive’s localizer holds the pose:
        Pose2d pose = drive.localizer.getPose();
        double robotHeading = pose.heading.toDouble();
        // (Alternatively, read directly from IMU: YawPitchRollAngles)

        // Rotate the joystick vector by –heading to get field-centric vector
        Vector2d input = Rotation2d.exp(-robotHeading).times(new Vector2d(fwd, str));


        // Build the motion command: input.x = field-forward component, input.y = field-strafe component
        PoseVelocity2d cmd = new PoseVelocity2d(new Vector2d(input.x, input.y), rot);

        drive.setDrivePowers(cmd);


        if (gamepad1.right_bumper) {
            shooter.shoot(1.0);
        } else {
            shooter.stop();
        }

        // Telemetry
        telemetry.addData("robotHeading (deg)", "%.1f", Math.toDegrees(robotHeading));
        telemetry.addData("fwd(str rotated)", "%.2f", input.x);
        telemetry.addData("str(strafe rotated)", "%.2f", input.y);
        telemetry.addData("rot", "%.2f", rot);

        // Optionally: pose
        Pose2d p = drive.localizer.getPose();
        telemetry.addData("pose x", "%.2f", p.position.x);
        telemetry.addData("pose y", "%.2f", p.position.y);
        telemetry.addData("pose heading (deg)", "%.1f", Math.toDegrees(p.heading.toDouble()));

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
    }

    private double applyDeadzone(double v, double dz) {
        if (Math.abs(v) < dz) return 0.0;
        return Math.copySign((Math.abs(v) - dz) / (1.0 - dz), v);
    }
}
