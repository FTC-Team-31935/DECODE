package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Shooter {
    private DcMotorEx shooterMotor0   = null;
    private DcMotorEx shooterMotor1   = null;
    private CRServo servo0;
    private CRServo servo1;
    private Boolean IsServoFlashRunning = false;
    private NormalizedColorSensor load_stopper;
    private Telemetry telemetry;

    public Shooter(HardwareMap hardwareMapInit, Telemetry  telemetry) {
        this.telemetry = telemetry;

        shooterMotor0 = hardwareMapInit.get(DcMotorEx .class, "motor0");
        shooterMotor1 = hardwareMapInit.get(DcMotorEx.class, "motor1");

        //shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);

        shooterMotor0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        servo0 = hardwareMapInit.get(CRServo.class, "servo0");
        servo1 = hardwareMapInit.get(CRServo.class, "servo1");
        servo1.setDirection(CRServo.Direction.REVERSE);

        load_stopper = hardwareMapInit.get(NormalizedColorSensor.class, "load_stopper");
        if (load_stopper instanceof SwitchableLight) {
            ((SwitchableLight)load_stopper).enableLight(true);
        }
    }

    public void loadTest (){

        if (load_stopper instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) load_stopper).getDistance(DistanceUnit.CM));
        }
        DistanceSensor asDS = (DistanceSensor) load_stopper;
        if (asDS.getDistance(DistanceUnit.CM) < 10) {
            servoOff();
        }
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
        shooterMotor0.setVelocity(-1200);
        shooterMotor1.setVelocity(1200);
    }

    public void shootMedium(){
        shooterMotor0.setVelocity(-1000);
        shooterMotor1.setVelocity(1000);
    }

    public void shootFast(){
        shooterMotor0.setVelocity(-970);
        shooterMotor1.setVelocity(970);
    }

    public void shootStop(){
        shooterMotor0.setVelocity(0);
        shooterMotor1.setVelocity(0);
    }
    public void GetShootSpeed(){
     telemetry.addData("Velocity0 (rps)", "%.3f", shooterMotor0.getVelocity());
     telemetry.addData("Velocity1 (rps)", "%.3f", shooterMotor1.getVelocity());
    }
    public void setVelocity(double velocity){
        shooterMotor0.setVelocity(-velocity);
        shooterMotor1.setVelocity(velocity+50);
    }
    public void increaseVelocity (double increase){
        double startVel=shooterMotor1.getVelocity();
        setVelocity(startVel+increase);
    }
}