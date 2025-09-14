package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor motor0   = null;
    private DcMotor motor1   = null;

    public Shooter(HardwareMap hardwareMapInit) {

        motor0 = hardwareMapInit.get(DcMotor .class, "motor0");
        motor1 = hardwareMapInit.get(DcMotor.class, "motor1");
        motor1.setDirection(DcMotor.Direction.REVERSE);
    }


    public void shootSlow(){
        motor0.setPower(.5);
        motor1.setPower(.5);
    }

    public void shootFast(){
        motor0.setPower(1);
        motor1.setPower(1);
    }

    public void shootMedium(){
        motor0.setPower(.75);
        motor1.setPower(.75);
    }

    public void shootStop(){
        motor0.setPower(0);
        motor1.setPower(0);
    }
}