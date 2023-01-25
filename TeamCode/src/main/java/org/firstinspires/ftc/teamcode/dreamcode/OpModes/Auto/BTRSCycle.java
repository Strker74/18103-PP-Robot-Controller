package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.Constants;

    @Autonomous
    public class BTRSCycle extends AutoTemplate {
        private double tile = 1;
        private double[] myJunc = {Constants.LOW_GOAL, Constants.MID_GOAL, Constants.HIGH_GOAL}; // An array of junction heights in an order to fit the cycle
        private double x = 1.65; // x position of the cone stack
        private double y = -1.6; // y position of the cone stack
        private double a = 0; // angle of robot to face the cone stack
        private void cycle(double d, int i){
            // once your at the cone stack
            path.add(() -> tilePointDrive(getEstimator().getX(), getEstimator().getY(), a));
            path.add(() -> tilePointDrive(x, getEstimator().getY(), a));
            path.add(() -> tilePointDrive(x, y, a));
            path.add(() -> setLiftPos(d));
            path.add(() -> tilePointDrive(x, y, a));
            path.add(() -> raiseLift());
            path.add(() -> tilePointDrive(x, y, 90));
            path.add(() -> tilePointDrive(x-.65, y, 90));
            path.add(() -> tilePointDrive(x-.65, y+tile*i+.65, 90));
            path.add(() -> tilePointDrive(x-.65, y+tile*i+.65, 90));
            path.add(() -> setLiftPos(myJunc[i]));
            path.add(() -> tilePointDrive(x-.75, y+tile*i+.65, 90));
            path.add(() -> lowerLift());
            path.add(() -> openClaw());
            path.add(() -> pause(1));
            path.add(() -> tilePointDrive(x-.65, y+tile*i+.65, 90));
            path.add(() -> setLiftDownA());
        }
        // once your at Junction/End of pre-place


    @Override
    public void buildPath() {
        setStartA(180);
        path.add(() -> tilePointDrive(0, -.1, 180));
        path.add(() -> closeClaw());
        path.add(() -> pause(1));
        // Grabs the cone in front of it ^
        path.add(() -> tilePointDrive(.575, -.1, 180));
        path.add(() -> pause(1));
        path.add(() -> setLiftMidA());
        path.add(() -> pause(2));
        // drives to junction and lifts the cone ^
        path.add(() -> tilePointDrive(.575, .025, 180));
        path.add(() -> pause(4));
        path.add(() -> setLiftLowA());
        path.add(() -> openClaw());
        path.add(() -> pause(2));
        // Places cone on junction ^
        path.add(() -> tilePointDrive(.575, -.1, 180));
        path.add(() -> setLiftDownA());
        path.add(() -> pause(2));
        path.add(() -> tilePointDrive(.575, -.1, 0));
        // backs away and turns around ^

        for(int i = 0; i <= 2; i++){
                cycle(Constants.CONESTACKTICKS[i], oppSideCyclePark()[i]);
                // uses the proper cone stack height for the slides, uses the array to place cones in wanted order
            }
            visionParking180(); // Parks based on vision
    }

}
