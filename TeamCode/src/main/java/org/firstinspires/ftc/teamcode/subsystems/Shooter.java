package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotor shooterMotor0   = null;
    private DcMotor shooterMotor1   = null;

    public Shooter(HardwareMap hardwareMapInit) {
        shooterMotor0 = hardwareMapInit.get(DcMotor .class, "motor0");
        shooterMotor1 = hardwareMapInit.get(DcMotor.class, "motor1");
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
    }

    public void shootSlow(){
        shooterMotor0.setPower(.5);
        shooterMotor1.setPower(.5);
    }

    public void shootMedium(){
        shooterMotor0.setPower(.75);
        shooterMotor1.setPower(.75);
    }

    public void shootFast(){
        shooterMotor0.setPower(1);
        shooterMotor1.setPower(1);
    }

    public void shootStop(){
        shooterMotor0.setPower(0);
        shooterMotor1.setPower(0);
    }
}