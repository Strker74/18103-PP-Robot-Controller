package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.Constants;

@Autonomous
public class BTRSCycle extends AutoTemplate {
    private double tile = 1;
    private double[] myJunc = {Constants.HIGH_GOAL, Constants.MID_GOAL, Constants.LOW_GOAL};
    private void cycle(double d, int i){
        double x = 1.65;
        double y = -1.6;
        double a = 0;

        // once your at the cone stack
        // path.add(() -> setLiftPos(d));
        // path.add(() -> tilePointDrive(x, y, a));
        // path.add(() -> raiseLift());
        // path.add(() -> tilePointDrive(x, y, 90));
        // path.add(() -> tilePointDrive(x-.65, y, 90));
        // path.add(() -> tilePointDrive(x-.65, y+tile*i+.65, 90));
        // path.add(() -> tilePointDrive(x-.65, y+tile*i+.65, 90));
        // path.add(() -> setLiftPos(myJunc[i]);
        // path.add(() -> tilePointDrive(x-.75, y+tile*i+.65, 90));
        // path.add(() -> lowerLift());
        // path.add(() -> openClaw());
        // path.add(() -> pause(1));
        // path.add(() -> tilePointDrive(x-.65, y+tile*i+.65, 90));
        // path.add(() -> tilePointDrive(x-.65, y+tile*i+.65, a));
        // path.add(() -> tilePointDrive(x, y+tile*i+.65, a));
        // path.add(() -> tilePointDrive(x, y, a));
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

