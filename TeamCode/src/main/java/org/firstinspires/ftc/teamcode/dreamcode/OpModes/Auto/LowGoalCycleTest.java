package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class LowGoalCycleTest extends AutoTemplate {

   @Override
   public void buildPath() {
      setStartA(0);
      path.add(() -> lift(210)); // drop 35 each cone

      path.add(() -> tilePointDrive(0,-0.1,0));
      path.add(this::closeClaw);
      path.add(() -> pause(.3));
      path.add(() -> tilePointDrive(0, 1, 0));
      path.add(this::openClaw);
      path.add(() -> tilePointDrive(0, .75, 0));
      // Cone Placed to terminal
      path.add(() -> tilePointDrive(2.1, 1, 0));
      path.add(this::closeClaw);
      path.add(this::setLiftMid);
      path.add(() -> tilePointDrive(2.1, 1, -135));
      path.add(this::setLiftLow);
      path.add(this::openClaw);
      path.add(this::setLiftMid);
      // Low Goal Cycle
      path.add(() -> tilePointDrive(1.1, 1, 0));
      if (getVisionAnalysis() == 1) {
         path.add(() -> tilePointDrive(1.25, 0, 0));
      } else if (getVisionAnalysis() == 2) {
         path.add(() -> tilePointDrive(1.1, -0.9, 0));
      }
   }
}
