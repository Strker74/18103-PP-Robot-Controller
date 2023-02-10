package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@Autonomous(group = "Park")
public class ParkBTRS extends AutoTemplate {
    double a = 0;
    double y = -.075;
    double x = 1.4;
    double da = 0;
    double dy = .01;
    double dx = .01;
    @Override
    public void buildPath() {
        setStartA(0);
        path.add(() -> setSpeed(DriveMode.Economy));
        path.add(() -> tilePointDrive(0, -0.075, 0));
        path.add(this::closeClaw);
        path.add(() -> pause(1));
        path.add(() -> setSpeed(DriveMode.Optimized));
        path.add(() -> tilePointDrive(x, y, a));
        path.add(() -> setSpeed(DriveMode.Economy));
        path.add(this :: setLiftMid);
        path.add(() -> pause(2));
        path.add(() -> tilePointDrive(x + 6 * dx, y + 8 * dy, a + da));
        path.add(() -> pause(2));
        path.add(this::openClaw);
        path.add(() -> setSpeed(DriveMode.Sport));
        path.add(() -> tilePointDrive(x + 6 * dx, y - .05, a + da));
        path.add(this::setLiftDownA);
        path.add(() -> tilePointDrive(x - .35, y, a + da));
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(x - .2, y + .85, a + da)); break;
            case 1: path.add(() -> tilePointDrive(x - .2, y, a + da + 90)); break;
            case 2: path.add(() -> tilePointDrive(x - .2, y - .85, a + da)); break;
        }
        //path.add(()-> tilePointDrive(1.3, -1.075, 0));
        //path.add(this::setLiftMid);
        //path.add(this::openClaw);
        //path.add(() -> pause(0.2));
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
