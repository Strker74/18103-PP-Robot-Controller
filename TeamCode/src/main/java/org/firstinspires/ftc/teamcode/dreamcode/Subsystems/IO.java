package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public class IO implements Subsystem {

    DcMotorEx liftLeft, liftRight;
    Servo left, right;
    final double LIFTPOSMIN = -100, LIFTPOSMAX = 950;
    double fixLift = .0055;
    final double kpuHold = 0.0075, kiHold = 0, kdHold = 0, kpdHold = 0.005,
            kvHold = 1/Motors.GoBILDA_312.getSurfaceVelocity(2),
            kaHold = 0, ksHold = 0.13, ksdHold = 0.65, liftPosHold = 0, liftPowHold = 0, biasHold = 0;
    double kpu = 0.0075, ki = 0, kd = 0, kpd = 0.005, kv = 1/Motors.GoBILDA_312.getSurfaceVelocity(2),
            ka = 0, ks = 0.13, ksd = 0.65, liftPos = 0, liftPow = 0, bias = 0;
    double integral = 0, derivative = 0;
    boolean isOff = true;
    double runningUpLowError = 0;

    public IO(DcMotorEx liftLeft, DcMotorEx liftRight, Servo left, Servo right) {
        this.liftLeft = liftLeft;
        this.liftRight = liftRight;
        this.left = left;
        this.right = right;
    }

    @Deprecated
    public boolean PIDLift(Profile p, double ty) {
        double e = (p.getSetPoint() - getLiftPos());
        if (Math.abs(e) > ty) {
            runLift(kpu * e);
            return false;
        } else {
            stop();
            return true;
        }
    }

    public boolean PTickLift(double ticks, double ty) {
        double e = (ticks - getLiftTickPos());
        if (e >= 0) {
            if (Math.abs(e) > ty) {
                runLift(kpu * e + ks);
                return false;
            } else {
                return true;
            }
        } else {
            if (Math.abs(e) > ty) {
                runLift(MathFx.scale(-0.15,1,kpd * e + ksd));
                return false;
            } else {
                return true;
            }
        }
    }

    public boolean PIDTickLift(double ticks, double ty, double dt) {
        double e = (ticks - getLiftTickPos());
        integral += e * dt;
        derivative = (e - derivative) / dt;
        if (e >= 0) {
            if (Math.abs(e) > ty) {
                runLift(kpu * e + ki * integral + kd * derivative + ks);
                derivative = e;
                return false;
            } else {
                derivative = 0;
                return true;
            }
        } else {
            if (Math.abs(e) > ty) {
                runLift(MathFx.scale(-0.15,1,kpu * e + ki *
                        integral + kd * derivative + ks));
                derivative = e;
                return false;
            } else {
                derivative = 0;
                return true;
            }
        }
    }

    @Deprecated
    public double getLiftPos() {
        return getLiftTickPos()*
                Motors.GoBILDA_312.getDistPerTicks(1);
    }

    public double getLiftTickPos() {
        return /*(liftRight.getCurrentPosition() + liftRight.getCurrentPosition())/2.*/ liftRight.getCurrentPosition() + bias;
    }

    public void PosAdjustLift(double pos) {
        liftPos += pos*Motors.GoBILDA_312.getTicksPerRev()/25;
        liftPos = MathFx.scale(LIFTPOSMIN, LIFTPOSMAX, liftPos);
    }

    public void runLift(double power) {
        if (getLiftTickPos() <= -100 && power < 0) {
            stop(); liftPow = 0;
        } else {
            liftLeft.setPower(power);
            liftRight.setPower(power);
            liftPow = power;
        }
    }

    //public void killPower() {liftPos = -100;}
    //public void dropToError() {double p = getTargetLiftPos() - getLiftTickPos(); if(p < -50){liftPos = p;}}

    public void openClaw() {
        left.setPosition(0.7);
        right.setPosition(0.3);
    }

    public void closeClaw() {
        left.setPosition(0.9);
        right.setPosition(0);
    }

    public void setLiftMid() {liftPos = 625;}

    public void raiseLift() {liftPos+=10;}

    public void lowerLift() {if(liftPos > 100+ LIFTPOSMIN) {liftPos-=10;}}

    public void raiseLift(int i) {liftPos+=(10*i);}

    public void lowerLift(int i) {if(liftPos > 100+LIFTPOSMIN){liftPos-=(10*i);}}

    public void setLiftHigh() {liftPos = 900;}

    public void setLiftLow() {liftPos = 400;}

    public void AutoPosAdjustLift(double pos) {
        PosAdjustLift(pos * 25/Motors.GoBILDA_312.getTicksPerRev());
    }

    public void raiseLift100() {liftPos += 100;}

    public void dropLift100() {liftPos -= 100;}

    public void setLiftDown() {
        liftPos = 0;
    }

    public double getTargetLiftPos() {
        return liftPos;
    }

    public void resetLift() {
        bias -= getLiftTickPos();
    }

    public void setLiftPos(double liftPos) {
        this.liftPos = liftPos;
    }

    public boolean IsOff() {
        return isOff;
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        telemetry.addData("Lift Power", liftPow);
        telemetry.addData("Lift Target Position", getTargetLiftPos());
        telemetry.addData("Lift Encoder Position", getLiftTickPos());
        telemetry.addData("Lift Error", getTargetLiftPos() - getLiftTickPos());
        gainScheduleKs();
        //isOff = PIDTickLift(liftPos,10);
        isOff = PIDTickLift(liftPos,10, dt);
    }

    public void gainScheduleKs() {
        if (getTargetLiftPos() == 0) {
            ksd = 0;
            ks = 0;
        } else {
            ksd = ksdHold;
            ks = ksHold;
        }
    }

    @Override
    public void stop() {
        setLiftDown();
        runLift(0);
    }

}
