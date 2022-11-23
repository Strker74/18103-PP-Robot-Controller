package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.dreamcode.States.OCV;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class VisionTest extends OpMode {

   OCV vision;
   OpenCvWebcam webcam;

   @Override
   public void init() {
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
      vision = new OCV(webcam);
   }

   @Override
   public void loop() {
      vision.update(0.01, telemetry);
      try {
         sleep(50);
      } catch (InterruptedException e) {
         e.printStackTrace();
      }
   }

}
