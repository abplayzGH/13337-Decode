package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

public class Turret extends Shooter {

    public Turret(HardwareMap hw, Telemetry telemetry) {
        super(hw, telemetry);

    }

    public Action setFlywheelRPM(double rpm) {
        return packet -> {
        setTargetVelocity(rpm);
        packet.put("Flywheel Target RPM", rpm);
            return true;
        };
    }
}