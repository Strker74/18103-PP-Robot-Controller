package org.firstinspires.ftc.teamcode.dreamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.control.Path;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public abstract class TeleOpTemplate extends Robot {

    int pathStep = 0;
    Double direction;
    Double dx;
    Double dy;
    Double da;
    Profile px;
    Profile py;
    ElapsedTime timer = new ElapsedTime();
    Path path = new Path(this::stopRobot);
    double tile = Constants.tile, startA = 0, x = 0, y = 0, a = 0;
    boolean increment = false;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        super.loop();
        tilePointDrive(x, y, a);
    }

    public void PIDDrive(double y, double x, double a) {
        this.x += x * Constants.xScale;
        this.y += y * Constants.yScale;
        this.a += a * Constants.aScale;
    }

    // Old
    @Deprecated
    public void drive(Profile profile, double tolerance) {
        if (direction == null) {
            direction = profile.getSetPoint() - super.getEstimator().getX();
        }
        if (super.getDrive().pathFollower(super.getEstimator(), profile, tolerance, timer.time(), direction/Math.abs(direction))) {
            pathStep++;
            direction = null;
            timer.reset();
        }
    }

    /**
     *
     * @param angle The desired angle in degrees
     * @param tolerance The tolerable error in degrees
     */
    @Deprecated
    public void turn(double angle, double tolerance) {
        angle = MathFx.radAngleWrap(Math.toRadians(angle)); // 180 - angle
        tolerance = Math.toRadians(tolerance);
        if (direction == null) {
            direction = MathFx.radAngleWrap(angle - super.getEstimator().getA());
        }
        if (super.getDrive().PIDTurn(super.getEstimator(), angle, tolerance, direction/Math.abs(direction))) {
            pathStep++;
            direction = null;
            timer.reset();
        }
    }

    public void pointDrive(double x, double y, double a, double tx, double ty, double ta) {
        a = MathFx.radAngleWrap(Math.toRadians(a + startA)); // 180 - a
        ta = Math.toRadians(ta);
        if (dx == null) {
            dx = y - super.getEstimator().getX();
            dy = x - super.getEstimator().getY();
            da = MathFx.radAngleWrap(a - super.getEstimator().getA());
            px = new TrapezoidalMotionProfile(y);
            py = new TrapezoidalMotionProfile(x);

        }
        if (super.getDrive().WaypointDrive(super.getEstimator(), px, py, a, tx, ty, ta,
                dx/Math.abs(dx), dy/Math.abs(dy), da/Math.abs(da), timer.time())) {
            pathStep++;
            dx = null;
            dy = null;
            da = null;
            timer.reset();
        }
    }

    public void pointDrive(double x, double y, double a) {
        pointDrive(x, y, a, 1, 1, 5);
    }

    public void tilePointDrive(double x, double y, double a) {
        pointDrive(x*tile*1.1, y*tile*1.1, a);
    }

    public void setStartA(double startA) {
        this.startA = startA;
    }

    public void lift(double pos, double ta) {
        if (super.getIo().getTargetLiftPos() != pos) {
            super.getIo().setLiftPos(pos);
        } else if (Math.abs(super.getIo().getLiftTickPos() - super.getIo().getTargetLiftPos()) < 12*ta) {
            pathStep++;
            timer.reset();
        }
    }

    public void lift(double pos) {
        lift(pos, 10);
    }




    public void liftIncrement(double adjustment) {
        if (!increment) {
            getIo().AutoPosAdjustLift(adjustment);
            increment = true;
        }
        if (super.getIo().IsOff()) {
            increment = false;
            pathStep++;
            timer.reset();
        }
    }

    public void pause(double lapse) {
        if (timer.seconds() > lapse) {
            pathStep++;
            timer.reset();
        }
    }

    public void setLiftLow() {lift(Constants.LOW_GOAL);}
    public void setLiftMid() {lift(Constants.MID_GOAL);}
    public void setLiftMidA() {lift(600);}
    public void setLiftHigh() {lift(Constants.HIGH_GOAL);}
    public void setLiftDownA() {lift(0);}
    //public void killPowerA() {super.getIo().killPower();}

    public void closeClaw() {
        super.getIo().closeClaw();
        pathStep++;
        timer.reset();
    }

    public void openClaw() {
        super.getIo().openClaw();
        pathStep++;
        timer.reset();
    }
}