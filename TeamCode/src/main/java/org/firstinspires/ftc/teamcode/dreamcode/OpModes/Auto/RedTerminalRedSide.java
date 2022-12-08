package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedTerminalRedSide extends AutoTemplate {

   @Override
   public void buildPath() {
      setStartA(90);
      path.add(() -> tilePointDrive(0, 1, 0));
      path.add(() -> tilePointDrive(0, 0, 90));
      path.add(() -> tilePointDrive(1.5,0 , 90));
      path.add(() -> getIo().setLiftHigh());
      //path.add(() -> tilePointDrive(2, 2, 90));
   }

}