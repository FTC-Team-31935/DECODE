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
import static org.firstinspires.ftc.teamcode.subsystems.ColorSelection.STARTING_LOCATION;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.ColorSelection;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ShooterB;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Oh Gnome Auto", group = "Auto")
//@Disabled
public class OhGnomeAuto extends LinearOpMode

{
    @Override public void runOpMode()
    {
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap ,telemetry);
        DigitalChannel digitalTouch = hardwareMap.get(DigitalChannel.class, "Load_Stopper");

        ShooterB shooter= new ShooterB(hardwareMap ,telemetry);
        ColorSelection colorSelection = new ColorSelection(this);
        colorSelection.selectColor();
        telemetry.clearAll();
        colorSelection.displayColor();
        mecanumDrive.resetIMU();



        waitForStart();

        while (opModeIsActive())
        {
            if (blackboard.get(STARTING_LOCATION)== "NEAR_BASKET") {
                mecanumDrive.moveRobot(-0.25, 0, 0);
                sleep(3500);
                mecanumDrive.moveRobot(0, 0, 0);
                sleep(250);


                shooter.shootMedium();
                sleep(500);
                for (int x = 1; x <= 4; x++) {
                    shooter.servoOn();
                    while (!digitalTouch.getState()) {
                        sleep(10);
                    }
                    shooter.servoOff();
                    sleep(300);
                }



                if (blackboard.get(ALLIANCE_KEY) == "RED") {
                    mecanumDrive.moveRobot(0, 0.25, 0);
                } else if (blackboard.get(ALLIANCE_KEY) == "BLUE") {
                    mecanumDrive.moveRobot(0, -0.25, 0);
                }
                sleep(1750);
                mecanumDrive.moveRobot(0, 0, 0);
                break;

            } else if (blackboard.get(STARTING_LOCATION)== "SMALL_TRIANGLE") {
                if (blackboard.get(ALLIANCE_KEY) == "RED") {
                    mecanumDrive.driveFieldRelative(0, -0.5, 0);
                    sleep(500);
                    mecanumDrive.driveFieldRelative(0, 0, 0);
                    sleep(300);
                    mecanumDrive.driveFieldRelative(0, 0, -0.5);
                    while (mecanumDrive.getYaw()<45) {
                        sleep(10);
                    }
                    mecanumDrive.driveFieldRelative(0, 0, 0);

                } else if (blackboard.get(ALLIANCE_KEY) == "BLUE") {
                    mecanumDrive.driveFieldRelative(0, 0.5, 0);
                    sleep(500);
                    mecanumDrive.driveFieldRelative(0, 0, 0);
                    sleep(300);

                    mecanumDrive.driveFieldRelative(0, 0, 0.5);
                    while (mecanumDrive.getYaw()>-45) {
                        sleep(10);
                    }
                    mecanumDrive.driveFieldRelative(0, 0, 0);

                }

                 sleep(1000);
                shooter.shootSmallTriangle();
                sleep(2000);
                for (int x = 1; x <= 4; x++) {
                    shooter.servoOn();
                    while (!digitalTouch.getState()) {
                        sleep(10);
                    }
                    shooter.servoOff();
                    sleep(300);
                }

               mecanumDrive.driveFieldRelative(.5, 0, 0);
                 sleep(200);
                mecanumDrive.driveFieldRelative(0, 0, 0);
                sleep(250);

                break;

            } else {
                //Do nothing
                break;
            }
        }
    }

}
