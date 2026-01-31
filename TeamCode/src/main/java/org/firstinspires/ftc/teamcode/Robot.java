package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.teamcode.vision.VisionManager;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@Config
public class Robot {
    private static final Robot inst = new Robot();

    public static int TARGET_TAG = 20;
    public static final String MOTOR_NAME = "turret_motor";

    public static double SHOOTER_READY_VELOCITY = 700;
    public static double LATCH_OPEN = 0.1;
    public static double LATCH_CLOSED = 0;

    public FtcDashboard dashboard;
    public Telemetry dashboardTelemetry;

    public Servo latch;
    public ColorSensor colorSensor;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public VisionManager vision;;
    public Mecanum mecanum;
    public final ElapsedTime matchTimer = new ElapsedTime();
    public MultipleTelemetry flightRecorder;


    public enum Mode {
        AUTO,
        TELEOP
    }

    public enum Alliance {
        RED, BLUE
    }

    public static Alliance alliance = Alliance.RED;
    public static Mode mode = Mode.AUTO;

    public Robot(){}

    public static Robot get(){
        return inst;
    }

    public void reset() {
        intake.stopIntake();
        turret.reset();
        shooter.setIdle();
        shooter.setRaw(0);
    }
    public Robot Init(Mode mode, HardwareMap hardwareMap, Telemetry telemetry) {
        Robot.mode = mode;

        latch = hardwareMap.get(Servo.class, "latchServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        /* ---------------- SUBSYSTEMS ---------------- */
        intake = new Intake();
        intake.init(hardwareMap);

        shooter = new Shooter(hardwareMap, telemetry);
        shooter.setMode(Shooter.Mode.RAW);

        turret = new Turret();
        turret.init(hardwareMap, MOTOR_NAME);

        WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam 1");
        vision = new VisionManager(hardwareMap, cam, new Size(640, 480)); //TODO Tune webcam

        mecanum = new Mecanum();
        mecanum.Init(hardwareMap);
        TARGET_TAG = (alliance == Alliance.RED) ? 24 : 20;

        latch.setPosition(0);
        vision.startDashboardStream(15); // TODO disable for comp
        matchTimer.reset();

        flightRecorder = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        reset();
        return inst;
    }

    public void endLoop() {

        flightRecorder.addData("ALLIANCE", alliance.toString());
        flightRecorder.addData("ENCODER VALUE", turret.getPosition());
        flightRecorder.addData("Color", colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
        flightRecorder.update();

    }
}
