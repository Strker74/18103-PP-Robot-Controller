package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@Autonomous(group = "Park")
public class ParkRTRS extends AutoTemplate {
   double a = 0;
   double y = .2;
   double x = 1.1;

   @Override
   public void buildPath() {
      setStartA(0);
      path.add(() -> setSpeed(DriveMode.Economy));
      path.add(this::closeClaw);
      path.add(() -> tilePointDrive(0, 1, a));
      path.add(this::openClaw);
      path.add(() -> setSpeed(DriveMode.Sport));
      path.add(() -> tilePointDriveUnscaled(0, y, a));
      path.add(() -> tilePointDriveUnscaled(x, y, a));
      //path.add(this::setLiftLow);
      switch(visionAnalysis){
         case 0:
            path.add(() -> tilePointDriveUnscaled(x, y + 1, a));
            break;
         case 1:
            path.add(() -> tilePointDriveUnscaled(x, y, a));
            path.add(() -> tilePointDrive(x, y, a + 90));
            break;
         case 2:
            path.add(() -> tilePointDriveUnscaled(x, y - 0.9, a));
            path.add(() -> tilePointDriveUnscaled(x, y - 0.9, a+90));
            path.add(() -> tilePointDriveUnscaled(x+.5, y - 0.9 - .15, a+90));
            break;
      }
   }
}