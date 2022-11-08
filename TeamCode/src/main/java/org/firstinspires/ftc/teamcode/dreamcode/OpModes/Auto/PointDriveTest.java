package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class PointDriveTest extends AutoTemplate {

    @Override
    public void buildPath() {
        path.add(() -> pointDrive(0, -1*tile, 270));
        path.add(() -> pointDrive(0, -2*tile, 270));
    }

}
