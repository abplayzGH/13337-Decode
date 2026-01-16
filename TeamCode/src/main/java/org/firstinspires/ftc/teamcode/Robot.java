package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.teamcode.vision.VisionManager;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

public class Robot {
    private static final int[] TARGET_TAGS = {20, 24};
    private static final String MOTOR_NAME = "turret_motor";

    private static final double SHOOTER_READY_VELOCITY = 700;
    private static final double LATCH_OPEN = 0.1;
    private static final double LATCH_CLOSED = 0;

    public FtcDashboard dashboard;
    public Telemetry dashboardTelemetry;

    public Servo latch;
    public ColorSensor colorSensor;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public VisionManager vision;;
    public Mecanum mecanum;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

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
    }
}
