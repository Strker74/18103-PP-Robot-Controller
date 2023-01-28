package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;

@Autonomous
public class ACycleBTRS extends AutoTemplate {
    private final double tile = 1;
    private final double[] myJunc = {Constants.LOW_GOAL, Constants.MID_GOAL, Constants.HIGH_GOAL}; // An array of junction heights in an order to fit the cycle
    private final double x = 1.65; // x position of the cone stack
    private final double y = -1.6; // y position of the cone stack
    private final double a = 0; // angle of robot to face the cone stack
    private final double pIFOJx =-.65;
    private final double pIFOJy = .65;
    private final double pIFOJa = 90;
    private void cycle(double d, int i){
        // once your at the cone stack
        path.add(() -> tilePointDrive(getEstimator().getX(), getEstimator().getY(), a));
        path.add(() -> tilePointDrive(x, getEstimator().getY(), a));
        path.add(() -> tilePointDrive(x, y, a));
        path.add(() -> setLiftPos(d));
        path.add(() -> tilePointDrive(x, y, a));
        path.add(() -> raiseLift());
        path.add(() -> tilePointDrive(x, y+.1, pIFOJa));
        path.add(() -> setLiftDownA());
        path.add(() -> tilePointDrive(x+pIFOJx, y+.1, pIFOJa));
        path.add(() -> tilePointDrive(x+pIFOJx, y+tile*i+ pIFOJy, pIFOJa));
        path.add(() -> tilePointDrive(x+pIFOJx, y+tile*i+ pIFOJy, pIFOJa));
        path.add(() -> setLiftPos(myJunc[i]));
        path.add(() -> tilePointDrive(x+pIFOJx-.1, y+tile*i+ pIFOJy, pIFOJa));
        path.add(() -> lowerLift());
        path.add(() -> openClaw());
        path.add(() -> pause(1));
        path.add(() -> tilePointDrive(x+pIFOJx, y+tile*i+pIFOJy, pIFOJa));
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
        path.add(() -> tilePointDrive(.65, -.1, 180));
        path.add(() -> pause(1));
        path.add(() -> setLiftLowA());
        path.add(() -> pause(2));
        // drives to junction and lifts the cone ^

        path.add(() -> tilePointDrive(.65, .02, 180));
        path.add(() -> pause(2));
        path.add(() -> lowerLift());
        path.add(() -> openClaw());
        path.add(() -> pause(1));
        // Places cone on junction ^
        path.add(() -> tilePointDrive(.575, 0, 180));
        path.add(() -> setLiftDownA());
        path.add(() -> pause(2));
        path.add(() -> tilePointDrive(1, 0, 180));
        //path.add(() -> tilePointDrive(.575, -.2, 0));
        // backs away and turns around ^

        //for(int i = 0; i <= 2; i++){
        //    cycle(Constants.CONESTACKTICKS[0], visionAnalysis);
            // uses the proper cone stack height for the slides, uses the array to place cones in wanted order
        //}
        visionParking180(); // Parks based on vision
    }

}

