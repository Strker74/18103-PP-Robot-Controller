package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;

public class IO implements Subsystem {

    DcMotorEx liftLeft, liftRight;
    Servo left, right;
    double kp = 0.0075, kpd = 0.008, kv = 1/Motors.GoBILDA_312.getSurfaceVelocity(2), ka = 0, ks = 0.13, ksd = 0.3, liftPos = 0;

    public IO(DcMotorEx liftLeft, DcMotorEx liftRight, Servo left, Servo right) {
        this.liftLeft = liftLeft;
        this.liftRight = liftRight;
        this.left = left;
        this.right = right;
    }

    public boolean PIDLift(Profile p, double ty) {
        double e = (p.getSetPoint() - getLiftPos());
        if (Math.abs(e) > ty) {
            runLift(kp * e);
            return false;
        } else {
            stop();
            return true;
        }
    }

    public boolean PIDTickLift(double ticks, double ty) {
        double e = (ticks - getLiftTickPos());
        if (Math.abs(e) > ty) {
            runLift(kp * e /*+ ks*/);
            return false;
        } else {
            return true;
        }
    }

    public double getLiftPos() {
        return getLiftTickPos()*
                Motors.GoBILDA_312.getDistPerTicks(1);
    }

    public double getLiftTickPos() {
        return liftLeft.getCurrentPosition();
    }

    public void PosAdjustLift(double pos) {
        liftPos += pos*Motors.GoBILDA_312.getTicksPerRev()/25;
    }

    public void runLift(double power) {
        if (getLiftTickPos() <= 0 && power < 0) {
            stop();
        } else {
            liftLeft.setPower(power);
            liftRight.setPower(power);
        }
    }

    public void openClaw() {
        left.setPosition(0.7);
        right.setPosition(0.3);
    }

    public void closeClaw() {
        left.setPosition(0.9);
        right.setPosition(0);
    }

    public void setLiftMid() {liftPos = 600;}

    public void setLiftHigh() {liftPos = 900;}

    public void setLiftLow() {liftPos = 300;}

    public void raiseLift() {liftPos += 100;}

    public void dropLift() {liftPos -= 100;}

    public void setLiftDown() {
        liftPos = 0;
    }

    public double getTargetLiftPos() {
        return liftPos;
    }


    @Override
    public void update(double dt, Telemetry telemetry) {
        telemetry.addData("Lift Target Position", getTargetLiftPos());
        telemetry.addData("Lift Encoder Position", getLiftTickPos());
        telemetry.addData("Lift Error", getTargetLiftPos() - getLiftTickPos());
        PIDTickLift(liftPos,10);
    }

    @Override
    public void stop() {
        runLift(0);
    }

    // 630 (Mid Goal)

}
