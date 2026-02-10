package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;

public class IntakeActions {

    Intake intake = new Intake();


    public IntakeActions(HardwareMap hardwareMap) {
        intake.init(hardwareMap);
    }

    public Action intakeOff() {
        return packet -> {
            intake.stopIntake();
            packet.put("Intake Status", "Off");
            return false;
        };
    }

    public Action intakeShoot() {
        return packet -> {
            intake.runIntake();
            intake.runTransfer();
            packet.put("Intake Status", "Shooting");
            return false;
        };
    }

    public Action intakeHold() {
        return packet -> {
            intake.runIntake();
            packet.put("Intake Status", "Holding");
            return false;
        };
    }
}