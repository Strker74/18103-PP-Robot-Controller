package org.firstinspires.ftc.teamcode.dreamcode.OpModes.Auto;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.control.Path;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

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

    public abstract void buildPath();

    @Override
    public void init() {
        super.init();
        buildPath();
    }

    @Override
    public void loop() {
        //telemetry.addData("pathStep", pathStep);
        super.loop();
        path.run(pathStep);
    }

    // Old
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