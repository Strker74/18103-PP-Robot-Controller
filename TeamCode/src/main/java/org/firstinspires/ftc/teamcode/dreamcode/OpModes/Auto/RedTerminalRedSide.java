package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedTerminalRedSide extends AutoTemplate {

   @Override
   public void buildPath() {
      setStartA(0);
      path.add(this::closeClaw);
      path.add(() -> tilePointDrive(0, 1.25, 0));
      path.add(this::openClaw);
      //path.add(this::setLiftLow);
      path.add(() -> tilePointDrive(2.25, 1.25, 90));
      path.add(() -> tilePointDrive(2.25, 1.25, 180));
      if (getVisionAnalysis() == 1) {
         path.add(() -> tilePointDrive(1.75, 0, 180));
      } else if (getVisionAnalysis() == 2) {
         path.add(() -> tilePointDrive(1.75, -1.25, 180));
      }
   }
}