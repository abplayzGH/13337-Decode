package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx IntakeMotor;

    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        IntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void runIntake() {
        IntakeMotor.setPower(1.0);
    }
    public void stopIntake() {
        IntakeMotor.setPower(0.0);
    }
}
