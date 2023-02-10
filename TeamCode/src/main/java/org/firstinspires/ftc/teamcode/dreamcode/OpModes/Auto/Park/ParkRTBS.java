package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@Autonomous(group = "Park")
        public class ParkRTBS extends AutoTemplate {
            double a = 0;
            double x = 1.6;
            double y = -.7;
            double da = 0;
            double dx = -.15;
            double dy = .0525;
            double tile = 0;
            @Override
            public void buildPath() {
                setStartA(a);
                path.add(() -> setSpeed(DriveMode.Economy));
                path.add(() -> tilePointDrive(0, -0.075, a));
                path.add(this::closeClaw);
                path.add(() -> pause(2));
                path.add(() -> tilePointDrive(0.08, -0.075, a));
                path.add(() -> setSpeed(DriveMode.Optimized));
                path.add(() -> tilePointDriveUnscaled(0, -1, a));
                path.add(this :: setLiftMid);
                path.add(() -> setSpeed(DriveMode.Sport));
                path.add(() -> tilePointDriveUnscaled(1.55, -1.11, a));
                path.add(() -> pause(1));
                path.add(this ::lowerMidLift);
                //path.add(() -> tilePointDrive(1.6, -1.05, a));
                path.add(this::openClaw);
                path.add(this ::setLiftMid);
                path.add(() -> tilePointDrive(1.55, -1.2, a)); // for testing
                path.add(() -> tilePointDrive(1, -1.1, a)); // for testing
                path.add(this :: setLiftDownA);
                switch(visionAnalysis){
                    case 0: path.add(() -> tilePointDrive(1, .85, a)); break;
                    case 1: path.add(() -> tilePointDrive(1.1, -.2, a)); break;
                    case 2: path.add(() -> tilePointDrive(1, -1.1, a)); break;
                }
    }
}

