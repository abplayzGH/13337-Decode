package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ballShooter {

    public DcMotorEx leftFlyWheel;
    public DcMotorEx rightFlyWheel;
    public DcMotorEx conveyorBelt;

    public void init(HardwareMap hardwareMap) {
        leftFlyWheel = hardwareMap.get(DcMotorEx.class, "leftFlyWheel");
        rightFlyWheel = hardwareMap.get(DcMotorEx.class, "rightFlyWheel");
        conveyorBelt = hardwareMap.get(DcMotorEx.class, "conveyorBelt");

        leftFlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyorBelt.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void shoot(double power) {
        leftFlyWheel.setPower(power);
        rightFlyWheel.setPower(power);
        conveyorBelt.setPower(0.5);
    }


}
