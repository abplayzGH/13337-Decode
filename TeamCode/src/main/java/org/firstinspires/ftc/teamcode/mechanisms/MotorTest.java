package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorTest {

    private DcMotor motor1;

    public void init(HardwareMap hwMap){
        motor1 = hwMap.get(DcMotor.class, "leftFront");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     *Speed double: values between -1.0 and 1.0
     */
    public void setMotorSpeed(double speed){
        motor1.setPower(speed);
    }

}


