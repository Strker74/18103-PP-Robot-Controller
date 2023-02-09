package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@Autonomous(group = "Park")
        public class ParkRTBS extends AutoTemplate {
            double a = 0;
            double x = 1.6;
            double y = -.7;
            double da = -15;
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
                path.add(() -> tilePointDriveUnscaled(1.6, -1.05, a));
                path.add(this :: setLiftMid);
                path.add(() -> setSpeed(DriveMode.Economy));
                path.add(() -> pause(3));
                path.add(() -> tilePointDrive(1.6, -.7, a + da));
                path.add(() -> tilePointDrive(1.6 - .15, -.7 + .0525, a + da));
                path.add(() -> tilePointDrive(1.6 - .15, -.7 + .0525-.1, a + da));
                path.add(() -> pause(2));
                //path.add(this ::lowerLift);
                ///path.add(() -> pause(3));
                path.add(this::openClaw);
                path.add(() -> pause(1.5));
                path.add(() -> tilePointDrive(1.6 - .15, -.7 + .0525-.25, a + da)); // for testing
                path.add(this :: setLiftDownA);
                path.add(() -> tilePointDrive(1.1, -.7 + .0525 - .25, a + da + 10));
                path.add(() -> setSpeed(DriveMode.Sport));
                y = 0;
                switch(visionAnalysis){
                    case 0:
                        path.add(() -> tilePointDriveUnscaled(1.1, y, a + da + 10));
                        tile = .9;
                    case 1:
                        path.add(() -> tilePointDriveUnscaled(1.1, y, a + da + 10 + 10));
                        path.add(() -> tilePointDriveUnscaled(1.1, y + tile, a + da + 10 + 10));
                    case 2:
                        break;
                }
                /*switch(visionAnalysis){
                    case 0:
                        path.add(() -> tilePointDriveUnscaled(1.1, .85, a + da + 10));
                        break;
                    case 1:
                        path.add(() -> tilePointDriveUnscaled(1.1, 0, a + da + 10));
                        path.add(() -> tilePointDriveUnscaled(1.1, 0, a + da + 100));
                        path.add(() -> tilePointDriveUnscaled(1.2, -.1, a + da + 100));
                        break;
                    case 2: break;
                }*/
    }
}

