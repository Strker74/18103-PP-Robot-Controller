package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;

public class IO implements Subsystem {

    DcMotorEx liftLeft, liftRight;
    Servo left, right;
    double kp = 0.5, kv = 1/ Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0;

    public IO(DcMotorEx liftLeft, DcMotorEx liftRight, Servo left, Servo right) {
        this.liftLeft = liftLeft;
        this.liftRight = liftRight;
        this.left = left;
        this.right = right;
    }

    public boolean PIDLift(Profile p, double ty) {
        double e = (p.getSetPoint() - getLiftPos());
        if (Math.abs(e) > ty) {
            double u = kp * e;
            runLift(u);
            return false;
        } else {
            stop();
            return true;
        }
    }

    public double getLiftPos() {
        return 0;
    }

    public void runLift(double power) {
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    public void openClaw() {
        left.setPosition(0.7);
        right.setPosition(0.3);
    }

    public void closeClaw() {
        left.setPosition(0.9);
        right.setPosition(0);
    }


    @Override
    public void update(double dt, Telemetry telemetry) {

    }

    @Override
    public void stop() {
        runLift(0);
    }

}
