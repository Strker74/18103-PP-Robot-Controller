package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@Autonomous(group = "Park")
public class ParkRTRS extends AutoTemplate {
   double a = 0;
   double y = .1;

   @Override
   public void buildPath() {
      setStartA(a);
      path.add(() -> setSpeed(DriveMode.Economy));
      path.add(this::closeClaw);
      path.add(() -> tilePointDrive(0, 1, a));
      path.add(this::openClaw);
      path.add(() -> setSpeed(DriveMode.Sport));
      path.add(() -> tilePointDriveUnscaled(0, y, a));
      path.add(() -> tilePointDriveUnscaled(1, y, a));
      //path.add(this::setLiftLow);
      visionParking(y, a);
   }
}