package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;

// X is forward
@Autonomous
public class VisionPark extends AutoTemplate {

    @Override
    public void buildPath() {
        setStartA(180);
        path.add(this::openClaw);
        path.add(() -> tilePointDrive(.6, 0, 180));
        path.add(() -> tilePointDrive(1, 0, 180));
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(1.25, 1, 180)); break;
            case 1: path.add(() -> tilePointDrive(1.25, 0, 180)); break;
            case 2: path.add(() -> tilePointDrive(1.25, -1, 180)); break;
        }
        telemetry.addData("Auton Done!", visionAnalysis);
        //path.add(this::killPowerA);
    }

}

