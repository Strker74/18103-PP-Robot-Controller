package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RTRSCycle extends AutoTemplate {

    @Override
    public void buildPath() {
        setStartA(0);
        path.add(this::closeClaw);
        path.add(() -> tilePointDrive(0, 1, 0));
        path.add(this::openClaw);
        path.add(() -> tilePointDrive(0, .75, 0));
        //path.add(this::setLiftLow); -> Set to highest cone height
        path.add(() -> tilePointDrive(2, 1, 0));
        sameSideCyclePark();
    }


}
