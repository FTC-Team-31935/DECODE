/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptBlackboard.ALLIANCE_KEY;
import static org.firstinspires.ftc.teamcode.subsystems.ColorSelection.DELAYED_START;
import static org.firstinspires.ftc.teamcode.subsystems.ColorSelection.STARTING_LOCATION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.ColorSelection;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ShooterB;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="Oh Gnome Auto2", group = "Auto")
//@Disabled
public class OhGnomeAuto2 extends LinearOpMode

{
    double DESIRED_DISTANCE = 40.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.1  ;   //  Forward Speed Control "Gain". e.g. Ramp update to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.05 ;   //  Strafe Speed Control "Gain".  e.g. Ramp update to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp update to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.2;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private double power = 0.5;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double  rangeError      = 5;
    double  headingError    = 5;
    double  yawError        = 5;
    MecanumDrive mecanumDrive;

    @Override public void runOpMode()
    {

        mecanumDrive = new MecanumDrive(hardwareMap ,telemetry);

        DigitalChannel digitalTouch = hardwareMap.get(DigitalChannel.class, "Load_Stopper");

        ShooterB shooter= new ShooterB(hardwareMap ,telemetry);
        ColorSelection colorSelection = new ColorSelection(this);
        colorSelection.selectColor();
        telemetry.clearAll();
        colorSelection.displayColor();
        mecanumDrive.resetIMU();
        AprilTagWebcam AprilTag = new AprilTagWebcam();
        AprilTag.init(hardwareMap,telemetry);

        waitForStart();

        float initialRuntime = (float) getRuntime();

        while (opModeIsActive())
        {

            int initialDelaySec = (int) blackboard.getOrDefault(DELAYED_START,0);
            long MilliDelay=(int)initialDelaySec* 1000L;
            sleep(MilliDelay);
            if (blackboard.get(STARTING_LOCATION)== "NEAR_BASKET") {
                shooter.shootMedium();
                mecanumDrive.moveRobot(-0.25, 0, 0);
                sleep(3500);
                mecanumDrive.moveRobot(0, 0, 0);
                sleep(250);

                if (blackboard.get(ALLIANCE_KEY) == "RED") {
                    DESIRED_TAG_ID = 24;

                    for (int x = 1; x <= 4; x++) {

                        telemetry.addLine("Start April tag search");
                        AprilTag.update();
                        desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                        AprilTag.displayDetetionTelemetry(desiredTag);

                        if (desiredTag != null) {

                            rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                            headingError = desiredTag.ftcPose.bearing;
                            yawError = desiredTag.ftcPose.yaw;
                        }else{
                            rangeError = 0;
                            headingError = 0;
                            yawError = 1;
                        }

                        while ((Math.abs(rangeError) > 1 || Math.abs(headingError) > 1 || Math.abs(yawError)> 1) && (getRuntime() - initialRuntime)<25.0){

                            telemetry.addLine("Running AutomaticMovement");

                            AprilTag.update();
                            desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                            AprilTag.displayDetetionTelemetry(desiredTag);

                            if (desiredTag != null) {
                                AutomaticMovement();
                            }else{
                                if ((getRuntime() - initialRuntime) > 20.0){
                                    YawCorrection(5);
                                }else{
                                    mecanumDrive.driveFieldRelative(0,0,-0.5);
                                }

                            }
                            telemetry.update();

                        }
                        mecanumDrive.moveRobot(0, 0, 0);
                        sleep(10);
                        shooter.servoOn();
                        while (!digitalTouch.getState()) {
                            sleep(10);
                        }
                        shooter.servoOff();
                        sleep(300);
                    }
                    mecanumDrive.moveRobot(0, 0.25, 0);

                } else if (blackboard.get(ALLIANCE_KEY) == "BLUE") {
                    DESIRED_TAG_ID = 20;

                    for (int x = 1; x <= 4; x++) {

                        telemetry.addLine("Start April tag search");
                        AprilTag.update();
                        desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                        AprilTag.displayDetetionTelemetry(desiredTag);
                        telemetry.update();
                        if (desiredTag != null) {

                            rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                            headingError = desiredTag.ftcPose.bearing;
                            yawError = desiredTag.ftcPose.yaw;
                        }else{
                            rangeError = 0;
                            headingError    = 0;
                            yawError = 1;

                        }
                        while ((Math.abs(rangeError) > 1 || Math.abs(headingError) > 1 || Math.abs(yawError)> 1) && (getRuntime() - initialRuntime)<25.0){

                            telemetry.addLine("Running AutomaticMovement");
                            telemetry.addLine("Error values:");
                            telemetry.addData("Range",rangeError);
                            telemetry.addData("Heading",headingError);
                            telemetry.addData("Yaw",yawError);

                            AprilTag.update();
                            desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                            AprilTag.displayDetetionTelemetry(desiredTag);

                            if (desiredTag != null) {
                                AutomaticMovement();
                            }else{
                                if ((getRuntime() - initialRuntime) > 20.0){
                                    YawCorrection(5);
                                }else{
                                    mecanumDrive.driveFieldRelative(0,0,0.1);
                                }

                            }
                        }
                        mecanumDrive.moveRobot(0, 0, 0);
                        sleep(10);
                        shooter.servoOn();
                        while (!digitalTouch.getState()) {
                            sleep(10);
                        }
                        shooter.servoOff();
                        sleep(300);
                    }
                    mecanumDrive.moveRobot(0, -0.25, 0);
                }
                sleep(2000);
                mecanumDrive.moveRobot(0, 0, 0);
                break;

            } else if (blackboard.get(STARTING_LOCATION)== "SMALL_TRIANGLE") {
                if (blackboard.get(ALLIANCE_KEY) == "RED") {
                    DESIRED_TAG_ID = 24;
                    mecanumDrive.driveFieldRelative(0, -0.5, 0);
                    sleep(500);
                    mecanumDrive.driveFieldRelative(0, 0, 0);
                    while (mecanumDrive.getYaw()<43) {
                        YawCorrection(45);
                    }
                    mecanumDrive.driveFieldRelative(0, -0.5, 0);
                    AprilTag.update();
                    desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                    telemetry.addData("desired tag",desiredTag!=null);
                    telemetry.addData("run time",(getRuntime() - initialRuntime));
                    telemetry.update();
                    while (desiredTag == null && (getRuntime() - initialRuntime) < (4.0 + initialDelaySec)){
                        sleep(10);
                        AprilTag.update();
                        desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                        AprilTag.displayDetetionTelemetry(desiredTag);
                        telemetry.addData("desired tag",desiredTag!=null);
                        telemetry.addData("run time",(getRuntime() - initialRuntime));
                        telemetry.update();
                    }
                    mecanumDrive.driveFieldRelative(0, 0, 0);


                    for (int x = 1; x <= 4; x++) {
                        telemetry.addLine("Start April tag search");
                        AprilTag.update();
                        desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                        AprilTag.displayDetetionTelemetry(desiredTag);
                        telemetry.update();
                        if (desiredTag != null) {

                            rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                            headingError = desiredTag.ftcPose.bearing;
                            yawError = desiredTag.ftcPose.yaw;
                        }else{
                            rangeError = 0;
                            headingError    = 0;
                            yawError = 1;

                        }
                        while ((Math.abs(rangeError) > 1 || Math.abs(headingError) > 1 || Math.abs(yawError)> 1) && (getRuntime() - initialRuntime)<25.0){
                            if (desiredTag != null) {
                                AutomaticMovement();
                            }else if ((getRuntime() - initialRuntime) > 20.0){
                                YawCorrection(5);
                            }
                            AprilTag.update();
                            desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                            AprilTag.displayDetetionTelemetry(desiredTag);
                            telemetry.update();
                        }
                        mecanumDrive.moveRobot(0, 0, 0);
                        shooter.shootMedium();
                        sleep(200);
                        shooter.servoOn();
                        while (!digitalTouch.getState()) {
                            sleep(10);
                        }
                        shooter.servoOff();
                        sleep(300);
                    }
                    mecanumDrive.driveFieldRelative(0, -0.5, 0);
                    sleep(700);
                    mecanumDrive.moveRobot(0, 0, 0);

                } else if (blackboard.get(ALLIANCE_KEY) == "BLUE") {
                    DESIRED_TAG_ID = 20;
                    mecanumDrive.driveFieldRelative(0, 0.5, 0);
                    sleep(500);
                    mecanumDrive.driveFieldRelative(0, 0, 0);
                    while (mecanumDrive.getYaw()>-43) {
                        YawCorrection(-45);
                    }



                    mecanumDrive.driveFieldRelative(0, 0.7, 0);
                    sleep(1550);

                    mecanumDrive.driveFieldRelative(0, 0.1, 0);
                    AprilTag.update();
                    desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                    telemetry.addData("desired tag",desiredTag!=null);
                    telemetry.addData("run time",(getRuntime() - initialRuntime));
                    telemetry.addLine("slidhing");
                    telemetry.update();

                    while (desiredTag == null && (getRuntime() - initialRuntime) < (15.0 + initialDelaySec)){
                        sleep(10);
                        AprilTag.update();
                        desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                        AprilTag.displayDetetionTelemetry(desiredTag);
                        telemetry.addData("desired tag",desiredTag!=null);
                        telemetry.addData("run time",(getRuntime() - initialRuntime));
                        telemetry.addLine("in loop");
                        telemetry.update();
                    }
                    sleep(200);
                    mecanumDrive.driveFieldRelative(0, 0, 0);
                    telemetry.addData("desired tag",desiredTag!=null);
                    telemetry.addData("run time",(getRuntime() - initialRuntime));
                    telemetry.addLine("done loop");
                    telemetry.update();
                    //sleep(4000);

                    for (int x = 1; x <= 4; x++) {
                        telemetry.addLine("Start April tag search");
                        AprilTag.update();
                        desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                        AprilTag.displayDetetionTelemetry(desiredTag);
                        telemetry.update();
                        if (desiredTag != null) {

                            rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                            headingError = desiredTag.ftcPose.bearing;
                            yawError = desiredTag.ftcPose.yaw;
                        }else{
                            rangeError = 0;
                            headingError    = 0;
                            yawError = 6;

                        }
                        shooter.shootMedium();
                        while ((Math.abs(rangeError) > 1.5 || Math.abs(headingError) > 5 || Math.abs(yawError)> 5 && (getRuntime() - initialRuntime)<25.0)){
                            if (desiredTag != null) {
                                AutomaticMovement();
                            }else{
                            YawCorrection(-45);
                            }
                            AprilTag.update();
                            desiredTag = AprilTag.getTagBySpecificId(DESIRED_TAG_ID);
                            AprilTag.displayDetetionTelemetry(desiredTag);
                            telemetry.update();
                        }
                        mecanumDrive.moveRobot(0, 0, 0);

                        shooter.servoOn();
                        while (!digitalTouch.getState()) {
                            sleep(10);
                        }
                        shooter.servoOff();
                        //sleep(3000);
                        sleep(300);
                    }
                    mecanumDrive.driveFieldRelative(0, -0.5, 0);
                    sleep(700);
                    mecanumDrive.moveRobot(0, 0, 0);

                }

                 /*sleep(1000);
                shooter.shootSmallTriangle();
                sleep(2000);
                for (int x = 1; x <= 4; x++) {
                    shooter.servoOn();
                    while (!digitalTouch.getState()) {
                        sleep(10);
                    }
                    shooter.servoOff();
                    sleep(300);
                }*/


               mecanumDrive.driveFieldRelative(.5, 0, 0);
               sleep(900);
               mecanumDrive.driveFieldRelative(0, 0, 0);
               sleep(250);

                break;

            } else {
                //Do nothing
                break;
            }
        }
    }
    private void AutomaticMovement() {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        headingError    = desiredTag.ftcPose.bearing;
        yawError        = desiredTag.ftcPose.yaw;


        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


        // Apply desired axes motions to the drivetrain.
        mecanumDrive.moveRobot(drive, strafe, turn);

        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    }
    private void YawCorrection(int desiredYaw) {
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        rangeError      = 0;
        headingError    = -(mecanumDrive.getYaw() - desiredYaw);
        yawError        = 0;


        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


        // Apply desired axes motions to the drivetrain.
        mecanumDrive.moveRobot(drive, strafe, turn);

        telemetry.addData("yaw coorec","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    }

}
