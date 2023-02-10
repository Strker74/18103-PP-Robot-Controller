package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@Autonomous(group = "Park")
public class ParkBTBS extends AutoTemplate {
    double a = 0;
    @Override
    public void buildPath() {
        setStartA(a);
        path.add(this::openClaw);
        path.add(() -> setSpeed(DriveMode.Optimized));
        path.add(() -> tilePointDrive(0, -1, a));
        path.add(() -> setSpeed(DriveMode.Sport));
        path.add(() -> tilePointDriveUnscaled(0, 0, a));
        path.add(() -> tilePointDriveUnscaled(1, 0, a));
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDriveUnscaled(.88, 1.05, a)); break;
            case 1: path.add(() -> tilePointDriveUnscaled(1, 0, 90)); break;
            case 2: path.add(() -> tilePointDriveUnscaled(1.2, -1, a)); break;
        }
        telemetry.addData("Auton Done!", visionAnalysis);
        //path.add(this::killPowerA);
    }

}

