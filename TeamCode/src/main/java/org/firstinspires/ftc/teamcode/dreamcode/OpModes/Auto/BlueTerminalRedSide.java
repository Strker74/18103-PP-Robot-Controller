package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueTerminalRedSide extends AutoTemplate {

    @Override
    public void buildPath() {
        setStartA(-90);
        super.getIo().closeClaw();
        super.getIo().setLiftLow();
        path.add(() -> tilePointDrive(-.25, 1, -90));
        super.getIo().openClaw();
        path.add(() -> tilePointDrive(1.3, 1, 0));
        for(int step = 1; step <= 3; step++) {
            path.add(() -> tilePointDrive(1.3, 2.5, 0));
            super.getIo().raiseLift(5-step);
            super.getIo().closeClaw();
            super.getIo().raiseLift100();
            path.add(() -> tilePointDrive(.75, 1.75, -90));
            super.getIo().setLiftLow();
            super.getIo().openClaw();
        }
        double park = 1.25;
        switch(visionAnalysis){
            case 0: park -= 0;break;
            case 1: park -= 1;break;
            case 2: park -= 2;break;
            //default: park -= .5; break;
        }
        double x = park;
        path.add(() -> tilePointDrive(x, 1.25, -90));
    }

}
