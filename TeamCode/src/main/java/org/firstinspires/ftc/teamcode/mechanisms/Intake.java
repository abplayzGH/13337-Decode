package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx IntakeMotor;
    public CRServo IntakeServo1;
    public CRServo IntakeServo2;

    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        IntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        IntakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        IntakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
        IntakeServo1.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeServo2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void runIntake() {

        IntakeMotor.setPower(1.0);
        IntakeServo1.setPower(1.0);
        IntakeServo2.setPower(1.0);
    }

    public void stopIntake() {
        IntakeMotor.setPower(0.0);
        IntakeServo1.setPower(0.0);
        IntakeServo2.setPower(0.0);
    }

    public void runOutTake() {
        IntakeMotor.setPower(-1.0);
        IntakeServo1.setPower(-1.0);
        IntakeServo2.setPower(-1.0);
    }
}
