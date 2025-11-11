package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ColorSelection
{
    private LinearOpMode myOpMode = null;
    public static final String ALLIANCE_KEY = "Alliance";
    public static final String DESIRED_TAG_ID = "Tag ID";
    public static final String STARTING_LOCATION = "location";

    public ColorSelection(LinearOpMode opMode){
        myOpMode = opMode;

    }
    public void selectColor(){
        myOpMode.telemetry.clearAll();

        while(!myOpMode.isStopRequested()){
            myOpMode.telemetry.addLine("choose red or blue");
            myOpMode.telemetry.addLine("push option button for blue");
            myOpMode.telemetry.addLine("push share button for red");
            myOpMode.telemetry.addLine();
            myOpMode.telemetry.addLine("push triangle button for small triangle starting spot");
            myOpMode.telemetry.addLine("push X button for near basket starting spot");
            myOpMode.telemetry.addLine();
            myOpMode.telemetry.addLine("push home/ps to select option");

            if (myOpMode.gamepad1.options){
                //blue
                blackboard.put(DESIRED_TAG_ID, 20);
                blackboard.put(ALLIANCE_KEY, "BLUE");
            }
            if (myOpMode.gamepad1.share) {
                //red
                blackboard.put(DESIRED_TAG_ID, 24);
                blackboard.put(ALLIANCE_KEY, "RED");
            }

            if (myOpMode.gamepad1.triangle){
                //small triangle
                blackboard.put(STARTING_LOCATION, "SMALL_TRIANGLE");
            }
            if (myOpMode.gamepad1.cross) {
                //near basket
                blackboard.put(STARTING_LOCATION, "NEAR_BASKET");
            }

            if (myOpMode.gamepad1.ps){
                break;
            }

           displayColor();
        }

        myOpMode.telemetry.clearAll();
    }

    public void displayColor(){
        if (blackboard.get(ALLIANCE_KEY) == "BLUE"){
            myOpMode.gamepad1.setLedColor(0,0,255,-1);
        }
        if (blackboard.get(ALLIANCE_KEY) == "RED") {
            myOpMode.gamepad1.setLedColor(255, 0, 0, -1);

        }
        myOpMode.telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
        myOpMode.telemetry.addData("Tag Id", blackboard.get(DESIRED_TAG_ID));
        myOpMode.telemetry.addData("starting location", blackboard.get(STARTING_LOCATION));
        myOpMode.telemetry.update();
    }

}

