package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous
public class BTBSOld extends AutoTemplate {

    @Override
    public void buildPath() {
        setStartA(0);
        path.add(this::closeClaw);
        path.add(() -> tilePointDrive(.5, 0, 135));
        path.add(() -> tilePointDrive(0, -1, 135));
        path.add(this::openClaw);
        path.add(() -> tilePointDrive(0, 0, 0));
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(1.5, 1, 0)); break;
            case 1: path.add(() -> tilePointDrive(1.5, 0, 0)); break;
            case 2: path.add(() -> tilePointDrive(1.5, -1, 0)); break;
        }
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
