package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveEstimators.IMU;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveEstimators.MKE;
import org.firstinspires.ftc.teamcode.dreamcode.States.OCV;
import org.firstinspires.ftc.teamcode.dreamcode.States.State;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.physics.Kinematic;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public class StateEstimator implements Subsystem, State {

    double a, a0, x0, y0, sI, s, sdot;
    Kinematic pos = new Kinematic(), vel = new Kinematic();
    IMU imu;
    MKE mke;
    OCV vision;
    boolean scanning = false;
    double[][] weights = new double[][] {{0, 1},
                                         {0, 1}};


    public StateEstimator(IMU imu, MKE mke) {
        this.imu = imu;
        this.mke = mke;
        a0 = 0;
        x0 = 0;
        y0 = 0;
        sI = 0;
        s = 0;
    }

    public StateEstimator(IMU imu, MKE mke, OCV vision, boolean scanning) {
        this.imu = imu;
        this.mke = mke;
        this.vision = vision;
        this.scanning = scanning;
        a0 = 0;
        x0 = 0;
        y0 = 0;
        sI = 0;
        s = 0;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        if (scanning) {
            vision.update(dt, telemetry);
            //telemetry.addData("Vision Cb: ", vision.getCb());
            //telemetry.addData("Vision Cr: ", vision.getCr());
            //telemetry.addData("Vision Y: ", vision.getY());
            telemetry.addData("Cb-Cr:", vision.getDiff());
            telemetry.addData("Park Position:", vision.getAnalysis().getName());
            try {
                sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        imu.update(dt, telemetry);
        mke.update(dt, telemetry);
        pos = Kinematic.dataFusion(new Kinematic[]{imu.getPos(), mke.getPos()}, weights);
        pos = pos.add(new Kinematic(x0, y0));
        a = MathFx.meanDataFusion(new double[]{mke.getA(), imu.getA()}, new double[]{1, 0}, a0);
        vel = new Kinematic(mke.getX_dot(), mke.getY_dot());
        telemetry.addData("X: ", pos.X());
        telemetry.addData("Y: ", pos.Y());
        telemetry.addData("A: ", Math.toDegrees(a));

        /*if (spinner != null) {
            s = MathFx.lowPassFilter(0.75, sI, spinner.getCurrentPosition() * Motors.GoBILDA_223.getDistPerTicks(96/Constants.mmPerInch));
            sdot = (s - sI)/dt;
            sI = s;
        }*/
    }

    @Override
    public void stop() {

    }

    public void zero() {
        a0 = -a;
        x0 = -pos.X();
        y0 = -pos.Y();
    }

    public double getX() {
        return pos.X();
    }

    public double getY() {
        return pos.Y();
    }

    public double getA() {
        return a;
    }

    public double getX_dot() {
        return vel.X();
    }

    public double getY_dot() {
        return vel.Y();
    }

    public double getS() {
        return s;
    }

    public double getSdot() {
        return sdot;
    }

}
