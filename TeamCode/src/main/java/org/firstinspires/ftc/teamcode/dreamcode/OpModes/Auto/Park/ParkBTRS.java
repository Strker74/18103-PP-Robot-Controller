package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;

@Autonomous(group = "Park")
public class ParkBTRS extends AutoTemplate {

    @Override
    public void buildPath() {
        setStartA(0);

        path.add(() -> tilePointDrive(0, -0.075, 0));
        path.add(this::closeClaw);
        path.add(() -> pause(1));
        path.add(() -> lift(200));
        path.add(() -> tilePointDrive(1.1, 0, 0));
        path.add(this::setLiftMid);
        path.add(()-> tilePointDrive(1.3, .4, 90));
        path.add(this::openClaw);
        path.add(() -> pause(0.2));
        /*
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(1.1, 1, 90)); break;
            case 1: path.add(() -> tilePointDrive(1.1, 0, 90)); break;
            case 2: path.add(() -> tilePointDrive(1.1, -1, 90)); break;
        }
        */
        //telemetry.addData("Auton Done!", visionAnalysis);
    }
}
