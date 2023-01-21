package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

        import static java.lang.Thread.sleep;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BTBSPark extends AutoTemplate {

    @Override
    public void buildPath() {
        setStartA(0);
        //path.add(this::openClaw);
        path.add(() -> tilePointDrive(0, -1, 0));
        path.add(() -> tilePointDrive(0, 0, 0));
        path.add(() -> tilePointDrive(1, 0, 0));
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(1.1, 1, 0)); break;
            case 1: path.add(() -> tilePointDrive(1.1, 0, 0)); break;
            case 2: path.add(() -> tilePointDrive(1.1, -1, 0)); break;
        }
        telemetry.addData("Auton Done!", visionAnalysis);
        //path.add(this::killPowerA);
    }

}

