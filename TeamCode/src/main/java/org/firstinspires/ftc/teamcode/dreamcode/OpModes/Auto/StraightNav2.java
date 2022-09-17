package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;

@Autonomous
public class StraightNav2 extends AutoTemplate {

    @Override
    public void init() {
        super.init();
        //path.add(() -> spin(0.5, 2));
        //path.add(() -> turn(45, 0.5));
        //path.add(() -> drive(p1, 1));
        //path.add(() -> pointDrive(-24, 24, 90));
        //path.add(() -> pointDrive(-48, 0, 180));
        path.add(() -> pointDrive(-24, -24, -90));
        path.add(() -> pointDrive(-24, -24, 0));
        path.add(() -> pointDrive(0, 0, 0));
    }

    @Override
    public void loop() {
        super.loop();
        path.run(pathStep);
    }

}
