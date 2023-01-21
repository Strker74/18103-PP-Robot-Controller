package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.control.Path;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

import java.util.ArrayList;

public abstract class AutoTemplate extends Robot {

    int pathStep = 0;
    Double direction;
    Double dx;
    Double dy;
    Double da;
    Profile px;
    Profile py;
    ElapsedTime timer = new ElapsedTime();
    Path path = new Path(this::stopRobot);
    double tile = Constants.tile, startA = 0;
    int visionAnalysis = 0;
    boolean increment = false;


    public abstract void buildPath();

    @Override
    public void init() {
        setScanning(true);
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        visionAnalysis = updateVisionAnalysis();
    }

    @Override
    public void start() {
        visionAnalysis = updateVisionAnalysis();
        buildPath();
    }

    @Override
    public void loop() {
        //telemetry.addData("pathStep", pathStep);
        super.loop();
        path.run(pathStep);
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
    public void setLiftLowA() {lift(350);}
    public void setLiftMidA() {lift(625);}
    public void setLiftPos(double d) {lift(d);}
    public void setLiftHigh() {lift(Constants.HIGH_GOAL);}
    public void setLiftDownA() {lift(0);}
    public void raiseLift() {getIo().raiseLift100();}
    public void lowerLift() {getIo().dropLift100();}
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

    public int updateVisionAnalysis() {
        return getEstimator().getVisionAnalysis();
    }

    public int getVisionAnalysis() {
        return visionAnalysis;
    }

    public void visionParking180(){

        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(1.25, 1, 180)); break;
            case 1: path.add(() -> tilePointDrive(1.25, 0, 180)); break;
            case 2: path.add(() -> tilePointDrive(1.25, -1, 180)); break;
        }
    }

    // Assumes setStartA(180);
    public void visionParkCycle() {
        switch(visionAnalysis){
            case 0: path.add(() -> tilePointDrive(2, 1, 0)); break;
            case 1: path.add(() -> tilePointDrive(2, 0, 0)); break;
            case 2: path.add(() -> tilePointDrive(2, -1, 0)); break;
        }
        //telemetry.addData("Auton Done!", visionAnalysis);
    }

    public void sameSideCyclePark() {
        ArrayList<Integer> points = new ArrayList<Integer>();
        for (int i = 2; i >= 0; i--) {
            if (i != visionAnalysis) {
                points.add(i);
            }
        }
        points.add(visionAnalysis);
        for (int i : points) { // access loop cycle using points.get(i)
            path.add(() -> tilePointDrive(2, 1 - i, 0));
            path.add(() -> tilePointDrive(2, 1, 0));
        }
    }

    public int[] oppSideCyclePark() {
        int[] points = new int[3];
        for (int i = 0; i <= 2; i--) {
            if (i != visionAnalysis) {
                points[i] = i;
            }
        }
        points[2] = visionAnalysis;
        return points;
    }

    /*public void spin(double pow, double time) {
        if (timer.seconds() < time) {
            super.getSpinner().spin(pow);
        } else {
            super.stopRobot();
            timer.reset();
            pathStep++;
        }
    }*/

}