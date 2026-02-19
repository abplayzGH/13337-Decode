package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//TODO Test this
public class MecanumDriveTest {
    public IMU imu;
    public GoBildaPinpointDriver odometry;
    public Servo lights;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public void init(HardwareMap hardwareMap) {
        // drive

        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Directions & modes
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double ODOMETRY_X_OFFSET = -102;
        double ODOMETRY_Y_OFFSET = -122;

        odometry.setOffsets(ODOMETRY_X_OFFSET, ODOMETRY_Y_OFFSET, DistanceUnit.MM);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odometry.resetPosAndIMU();

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    public void drive(double y, double x, double rx, double accelerator) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double accelMultiplier = Math.min(1.0 - 0.7 * accelerator, 1.0);
        x *= 1.1;

        double flPower = accelMultiplier * ((y + x + rx) / denominator);
        double blPower = accelMultiplier * ((y - x + rx) / denominator);
        double frPower = accelMultiplier * ((y - x - rx) / denominator);
        double brPower = accelMultiplier * ((y + x - rx) / denominator);

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);
    }

    public void resetFieldCentric() {
        imu.resetYaw();
    }


    public void driveFieldCentric(double y, double x, double rx, double accelerator) {
        double imuBotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // 2. Standard Rotation Matrix for Field Centric
        double rotX = x * Math.cos(-imuBotHeading) - y * Math.sin(-imuBotHeading);
        double rotY = x * Math.sin(-imuBotHeading) + y * Math.cos(-imuBotHeading);

        // 3. Pass to the standard drive method
        drive(rotY, rotX, rx, accelerator);
    }

    public double getPosX() {
        return odometry.getPosX(DistanceUnit.INCH);
    }

    public double getPosY() {
        return odometry.getPosY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return odometry.getHeading(AngleUnit.DEGREES);
    }

    public void setHeading(double heading) {
        odometry.setHeading(heading, AngleUnit.DEGREES);
    }

    public void setOdometryXY(double x, double y) {
        odometry.setPosX(x, DistanceUnit.INCH);
        odometry.setPosY(y, DistanceUnit.INCH);
    }

    public void odometryUpdate(double radians) {

        // This allows us to tell the IMU/Odometry that "Forward" is
        // a different direction based on where we started.
        imu.resetYaw();
        // Note: If using GoBilda Pinpoint, you can also use:
        odometry.update();
        odometry.setHeading(radians, AngleUnit.RADIANS);
    }
    }

}
