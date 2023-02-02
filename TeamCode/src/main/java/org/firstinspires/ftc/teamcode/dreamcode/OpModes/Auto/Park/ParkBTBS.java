package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.Park;

        import static java.lang.Thread.sleep;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

        import org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto.AutoTemplate;

@Autonomous(group = "Park")
public class ParkBTBS extends AutoTemplate {
    double a = 0;
    @Override
    public void buildPath() {
        setStartA(a);
        path.add(this::openClaw);
        path.add(() -> tilePointDrive(0, -1, a));
        path.add(() -> tilePointDrive(0, 0, a));
        path.add(() -> tilePointDrive(1, 0, a));
        visionParking(0, a);
        //telemetry.addData("Auton Done!", visionAnalysis);
        //path.add(this::killPowerA);
    }

}

