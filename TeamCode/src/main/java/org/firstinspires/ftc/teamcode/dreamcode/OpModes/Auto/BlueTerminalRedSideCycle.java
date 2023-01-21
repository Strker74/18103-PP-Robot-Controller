package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
public class BlueTerminalRedSideCycle extends AutoTemplate {

    private void cycle(double d, int i){
        path.add(() -> tilePointDrive(.65, -.1, 0));
        path.add(() -> tilePointDrive(.65, -1.6, 0));
        path.add(() -> tilePointDrive(1.65, -1.6, 0));
        // path.add(() -> setLiftPos(d));
        // path.add(() -> tilePointDrive(1.65, -1.7, 0));
        // path.add(() -> raiseLift());
        // path.add(() -> tilePointDrive(1.65, -1.6, 0));
        // path.add(() -> tilePointDrive(1.65, -1.6, 180));
    }
    @Override
    public void buildPath() {
        setStartA(180);
        path.add(() -> tilePointDrive(0, -.1, 180));
        path.add(() -> closeClaw());
        path.add(() -> pause(1));
        path.add(() -> tilePointDrive(.575, -.1, 180));
        path.add(() -> pause(1));
        path.add(() -> setLiftMidA());
        path.add(() -> pause(2));
        path.add(() -> tilePointDrive(.575, .025, 180));
        path.add(() -> pause(4));
        path.add(() -> setLiftLowA());
        path.add(() -> openClaw());
        path.add(() -> pause(2));
        path.add(() -> tilePointDrive(.575, -.1, 180));
        path.add(() -> setLiftDownA());
        path.add(() -> pause(2));
        path.add(() -> tilePointDrive(.575, -.1, 0));



        //visionParking180();



        /*
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
            }*/
    }
}

