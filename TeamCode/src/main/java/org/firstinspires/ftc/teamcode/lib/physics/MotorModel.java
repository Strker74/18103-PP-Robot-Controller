package org.firstinspires.ftc.teamcode.lib.physics;

import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public class MotorModel implements Model {

    Motors motorVersion;
    double maxP, maxA, maxV;
    double a, v, p;
    double eff, gr, kT, R, kV, J;

    public static void main(String[] args) {
        MotorModel motor = new MotorModel(Motors.GoBILDA_435,100, 2d, 0.9, 2./3);
        //TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(5*24, .8 * motor.maxV, motor.maxA * .8);

        for (int i = 0; i <= 6; i++) {
            System.out.print(i + " ");
            motor.run(12 * ((i <= 3) ? 1 : -1), 1);
            System.out.println(motor.v + " " + motor.p);
            //prev_time = timeStamp;
        }

        System.out.println("Position: " + motor.p*Constants.inPerM + " " + 5*24);
        System.out.println("Velocity: " + motor.v + " " + 0);
        System.out.println("Acceleration: " + motor.a + " " + 0);
    }

    public MotorModel(Motors motorVersion, double maxA, double J,
                      double eff, double gr) {
        this.motorVersion = motorVersion;
        maxP = 12;
        maxV = motorVersion.getMaxAngularVelocity();
        this.maxA = maxA;
        reset();
        this.eff = eff;
        this.gr = gr;
        kT = (motorVersion.getStallTorque()*Constants.kgcmtoNm*gr*eff)/(motorVersion.getStallCurrent());
        R = maxP/ motorVersion.getStallCurrent();
        kV = (maxP - motorVersion.getFreeCurrent()*R)/maxV;
        this.J = J;
    }

    public void run(double pow, double dt) {
        a = MathFx.scale(-maxA, maxA, accelerate(pow));
        v = MathFx.scale(-maxV, maxV, v + a * dt);
        p += v * dt;
    }

    public double accelerate(double pow) {
        pow = MathFx.scale(-maxP, maxP, pow);
        return eff*gr*kT*(pow - gr*v*kV)/(R * J);
    }

    public void reset() {
        a = 0;
        v = 0;
        p = 0;
    }

    public double getA() {
        return a;
    }

    public double getP() {
        return p;
    }

    public double getV() {
        return v;
    }

}
