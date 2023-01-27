package org.firstinspires.ftc.teamcode.dreamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dreamcode.Constants;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;
import org.firstinspires.ftc.teamcode.lib.drivers.Motors;
import org.firstinspires.ftc.teamcode.lib.motion.Profile;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;

public class Drive implements Subsystem {

    //DriveMode mode = DriveMode.Sport;
    //double y, x, turn;

    DcMotorEx fl;
    DcMotorEx fr;
    DcMotorEx bl;
    DcMotorEx br;
    DcMotorEx[] driveMotors;
    double kp = 0.5, kv = 1/Motors.GoBILDA_435.getSurfaceVelocity(2), ka = 0, kpa = 4, kp1 = 3;

    public Drive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        driveMotors = new DcMotorEx[]{fl, fr, bl, br};
    }

    @Override
    public void update(double dt, Telemetry telemetry) {
        //telemetry.addData("Mode", mode.getName());
    }

    @Override
    public void stop() {
        setDriveMotors(0);
    }

    /*public void changeMode(){
        if(mode == DriveMode.Sport){mode = DriveMode.Optimized;}
        if(mode == DriveMode.Optimized){mode = DriveMode.Sport;}
        POVMecanumDrive(y, x, turn, mode);
    }*/
    /**
     * Sets Drive to go forward/backwards
     * @param power Speed of movement
     */
    public void setDriveMotors(double power) {
        for (DcMotorEx i : driveMotors) {
            i.setPower(power);
        }
    }

    /**
     * Sets Drive to go left/right
     * @param power Speed of movement
     */
    public void setStrafeMotors(double power) {
        fl.setPower(power);
        bl.setPower(-power);
        fr.setPower(-power);
        br.setPower(power);
    }

    /**
     * Sets Drive to rotate
     * @param power Speed of Movement
     */
    public void setRotateMotors(double power) {
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(-power);
        br.setPower(-power);
    }

    /**
     * Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void POVMecanumDrive(double y, double x, double turn, DriveMode mode) {
        //this.mode = mode;
        //this.y = y;
        //this.x = x;
        //this.turn = turn;
        double v1 = -(-y - (turn * Constants.turnScale) - (x/Constants.strafeScale));
        double v2 = -(-y - (turn * Constants.turnScale) + (x/Constants.strafeScale));
        double v3 = -(-y + (turn * Constants.turnScale) - (x/Constants.strafeScale));
        double v4 = -(-y + (turn * Constants.turnScale) + (x/Constants.strafeScale)) *
                Motors.GoBILDA_435.getRPM()/Motors.GoBILDA_312.getRPM();


        double v = Math.max(Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.abs(v3)), Math.abs(v4));
        if (v > 1) {
            v1 /= v;
            v2 /= v;
            v3 /= v;
            v4 /= v;
        }

        fl.setPower(v1 * mode.getScaling());
        bl.setPower(v2 * mode.getScaling());
        br.setPower(v3 * mode.getScaling());
        fr.setPower(v4 * mode.getScaling());
    }


    /**
     * Field-Centric Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void FCMecanumDrive(double y, double x, double turn, DriveMode mode, StateEstimator robot) {
        double x2 = x * Math.cos(robot.getA()) - y * Math.sin(robot.getA());
        double y2 = x * Math.sin(robot.getA()) + y * Math.cos(robot.getA());
        POVMecanumDrive(y2, x2, turn, mode);
    }


    // Old
    public boolean pathFollower(StateEstimator robot, Profile profile, double tolerance, double time, double direction) {
        double setPoint = profile.getSetPoint();
        if ((setPoint - robot.getX())*direction > tolerance) {
            double Pe = profile.getPosition(time) - robot.getX();
            double Ve = profile.getVelocity(time) - robot.getX_dot();
            double Ae = 0;//profile.getAcceleration(time) - super.getEstimator().getY_dDot();
            double u = kp * Pe + kv * Ve + ka * Ae;
            setDriveMotors(u);
            return false;
        } else {
            stop();
            return true;
        }
    }

    public boolean PIDTurn(StateEstimator robot, double setPoint, double tolerance, double direction) {
        double e = MathFx.radAngleWrap(setPoint - robot.getA());
        if (e*direction > tolerance) {
            double u = kpa * e;
            setRotateMotors(u);
            return false;
        } else {
            stop();
            return true;
        }
    }

    public boolean WaypointDrive(StateEstimator robot, Profile xp, Profile yp, double af, double tx,
                                 double ty, double ta, double dx, double dy, double da, double time) {
        double ea = MathFx.radAngleWrap(af - robot.getA());
        double exp = (xp.getSetPoint() - robot.getX());
        double eyp = (yp.getSetPoint() - robot.getY());
        //double exv = (xp.getVelocity(time) - robot.getX_dot());
        //double eyv = (yp.getVelocity(time) - robot.getY_dot());
        if (ea*da > ta || exp*dx > tx || eyp*dy > ty) {
            double ua = kpa * ea;
            double ux = kp1 * exp /*+ kv * exv*/;
            double uy = kp1 * eyp /*+ kv * eyv*/;
            FCMecanumDrive(ux, uy, ua, DriveMode.Optimized, robot);
            return false;
        } else {
            stop();
            return true;
        }
    }

    public double getFlPow() {
        return fl.getPower();
    }

    public double getFrPow() {
        return fr.getPower();
    }

    public double getBlPow() {
        return bl.getPower();
    }

    public double getBrPow() {
        return br.getPower();
    }



}
