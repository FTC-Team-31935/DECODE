package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Shooter {
    private DcMotor shooterMotor0   = null;
    private DcMotor shooterMotor1   = null;
    private CRServo servo0;
    private CRServo servo1;
    private Boolean IsServoFlashRunning = false;



    public Shooter(HardwareMap hardwareMapInit) {
        shooterMotor0 = hardwareMapInit.get(DcMotor .class, "motor0");
        shooterMotor1 = hardwareMapInit.get(DcMotor.class, "motor1");
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);

        servo0 = hardwareMapInit.get(CRServo.class, "servo0");
        servo1 = hardwareMapInit.get(CRServo.class, "servo1");
        servo1.setDirection(CRServo.Direction.REVERSE);
    }
    public void servoOn (){
        servo0.setPower(1);
        servo1.setPower(1);
    }
    public void servoOff (){
        servo0.setPower(0);
        servo1.setPower(0);
    }
    public void servoFlash (){
        if (IsServoFlashRunning){
            return;
        }
        IsServoFlashRunning = true;
        try {
            servoOn();
            Thread.sleep(500) ;
            servoOff();
            Thread.sleep(1000) ;
            servoOn();
            Thread.sleep(500) ;
            servoOff();
            Thread.sleep(1000) ;
            servoOn();
            Thread.sleep(500) ;
            servoOff();
            Thread.sleep(1000) ;
        IsServoFlashRunning = false;
        } catch (InterruptedException e) {
        }
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