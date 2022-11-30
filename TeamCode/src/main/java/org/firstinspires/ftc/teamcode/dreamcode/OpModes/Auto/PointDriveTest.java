package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class PointDriveTest extends AutoTemplate {

    @Override
    public void buildPath() {
        path.add(() -> tilePointDrive(0, 2, 90));
        path.add(() -> tilePointDrive(2, 2, 90));
    }

}
