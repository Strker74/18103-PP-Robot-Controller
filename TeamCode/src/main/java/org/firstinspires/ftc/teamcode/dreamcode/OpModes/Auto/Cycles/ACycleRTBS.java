package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;

@Autonomous
public class ACycleRTBS extends AutoTemplate {
    @Override
    public void buildPath() {
        setStartA(180);
        double y = -0.9, yd = -0.065;
        path.add(() -> tilePointDrive(0, yd, 180));
        path.add(() -> closeClaw());
        path.add(() -> pause(1));
        // Grabs the cone in front of it ^
        path.add(() -> tilePointDrive(0, yd + y, 180));
        path.add(() -> tilePointDrive(.65, yd + y, 180));
        path.add(() -> pause(1));
        path.add(() -> setLiftLowA());
        path.add(() -> pause(2));
        // drives to junction and lifts the cone ^

        path.add(() -> tilePointDrive(.65, .15+ yd + y, 180));
        path.add(() -> pause(2));
        path.add(() -> lowerLift());
        path.add(() -> openClaw());
        path.add(() -> pause(1));
        // Places cone on junction ^
        path.add(() -> tilePointDrive(.575, yd + y, 180));
        path.add(() -> setLiftDownA());
        path.add(() -> pause(2));
        path.add(() -> tilePointDrive(1, yd + y, 180));
        //path.add(() -> tilePointDrive(.575, -.2, 0));
        // backs away and turns around ^

        //for(int i = 0; i <= 2; i++){
        //    cycle(Constants.CONESTACKTICKS[0], visionAnalysis);
        // uses the proper cone stack height for the slides, uses the array to place cones in wanted order
        //}
        visionParking2(); // Parks based on vision
    }


}
