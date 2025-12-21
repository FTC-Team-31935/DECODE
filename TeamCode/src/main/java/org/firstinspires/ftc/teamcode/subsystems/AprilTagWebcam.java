package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

   private AprilTagProcessor aprilTagProcessor;

   private VisionPortal visionPortal;

   private List<AprilTagDetection> detectedTags = new ArrayList<>();

   private Telemetry telemetry;




    public void init(HardwareMap hwMap, Telemetry telemetry) {
       this.telemetry = telemetry;

       aprilTagProcessor = new AprilTagProcessor.Builder()
               .setDrawTagID(true)
               .setDrawTagOutline(true)
               .setDrawAxes(true)
               .setDrawCubeProjection(true)
               .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
               .build();

       // Adjust Image Decimation to trade-off detection-range for detection-rate.
       // e.g. Some typical detection data using a Logitech C920 WebCam
       // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
       // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
       // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
       // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
       // Note: Decimation can be changed on-the-fly to adapt during a match.
       aprilTagProcessor.setDecimation(2);

       VisionPortal.Builder builder = new VisionPortal.Builder();
       builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
       builder.setCameraResolution(new Size(640 , 480));
       builder.addProcessor(aprilTagProcessor);

       visionPortal = builder.build();
   }

   public void update() {
       detectedTags = aprilTagProcessor.getDetections();
   }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetetionTelemetry(AprilTagDetection detectedId) {
       if (detectedId == null) {return;}
       if (detectedId.metadata !=null) {
           telemetry.addData("Found", "ID %d (%s)", detectedId.id, detectedId.metadata.name);
           telemetry.addData("Range",  "%5.1f inches", detectedId.ftcPose.range);
           telemetry.addData("Bearing","%3.0f degrees", detectedId.ftcPose.bearing);
           telemetry.addData("Yaw","%3.0f degrees", detectedId.ftcPose.yaw);}
    }

    public AprilTagDetection getTagBySpecificId(int id) {
       for (AprilTagDetection detection : detectedTags) {
           if (detection.id == id){
               return detection;
           }
       }
       return null;
    }

    public void stop() {
       if (visionPortal != null) {
           visionPortal.close();
       }
    }
}
