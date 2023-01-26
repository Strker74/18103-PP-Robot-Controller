package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;

@Autonomous
public class RTRSPark extends AutoTemplate {

   @Override
   public void buildPath() {
      setStartA(0);
      path.add(this::closeClaw);
      path.add(() -> tilePointDrive(0, 1, 0));
      path.add(this::openClaw);
      path.add(() -> tilePointDrive(0, .75, 0));
      //path.add(this::setLiftLow);
      path.add(() -> tilePointDrive(1.1, 1, 0));
      if (getVisionAnalysis() == 1) {
         path.add(() -> tilePointDrive(1.25, 0, 0));
      } else if (getVisionAnalysis() == 2) {
         path.add(() -> tilePointDrive(1.1, -0.9, 0));
      }
   }
}