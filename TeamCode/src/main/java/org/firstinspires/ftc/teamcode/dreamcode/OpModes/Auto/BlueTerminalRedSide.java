package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueTerminalRedSide extends AutoTemplate {

    @Override
    public void buildPath() {
        setStartA(180);
        path.add(this::closeClaw);
        /*try{
            sleep(2000);
        } catch (Exception e){
            e.printStackTrace();
        }*/

        path.add(() -> tilePointDrive(.6, 0, 180));
        path.add(this::setLiftMidA);
        path.add(() -> tilePointDrive(.6, .1, 180));
        //path.add(this::setLiftLow);
        path.add(this::openClaw);
        path.add(() -> tilePointDrive(.6, 0, 180));
        path.add(() -> tilePointDrive(1, 0, 180));
        //super.getIo().setLiftDown();
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(1.25, 1, 180)); break;
            case 1: path.add(() -> tilePointDrive(1.25, 0, 180)); break;
            case 2: path.add(() -> tilePointDrive(1.25, -1, 180)); break;
        }
        telemetry.addData("Auton Done!", visionAnalysis);
        path.add(this::setLiftDownA);
    }
}
