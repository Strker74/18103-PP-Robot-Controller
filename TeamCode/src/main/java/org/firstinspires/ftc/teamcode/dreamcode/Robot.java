package org.firstinspires.ftc.teamcode.dreamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveEstimators.IMU;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveEstimators.MKE;
import org.firstinspires.ftc.teamcode.dreamcode.States.OCV;
import org.firstinspires.ftc.teamcode.dreamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.dreamcode.Subsystems.IO;
import org.firstinspires.ftc.teamcode.dreamcode.Subsystems.StateEstimator;
import org.firstinspires.ftc.teamcode.dreamcode.Subsystems.Subsystem;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot extends OpMode {

    Subsystem[] subsystems;

    DcMotorEx fl, fr, bl, br, lift1, lift2;
    DcMotorEx[] driveMotors;
    Servo leftClaw, rightClaw;
    OpenCvWebcam webcam;
    BNO055IMU imu;
    Drive drive;
    IO io;
    StateEstimator estimator;
    ElapsedTime timer;
    double dt;
    boolean scanning = false;

    @Override
    public void init() {
        initDrive();
        initIO();
        //initSpinner();
        initStateEstimator();
        subsystems = new Subsystem[]{drive, io,/* spinner,*/ estimator};
        timer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        dt = timer.seconds();
        timer.reset();
        getEstimator().update(getDt(), telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        dt = timer.seconds();
        timer.reset();
        for (Subsystem system: subsystems) {
            system.update(getDt(), telemetry);
        }
        telemetry.update();
    }

    public void initDrive() {
        fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        br = hardwareMap.get(DcMotorEx.class, "backRight");

        //fr.setDirection(DcMotorEx.Direction.REVERSE);
        //br.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{fl, fr, bl, br};

        for (DcMotorEx motor : driveMotors) {
            //motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        drive = new Drive(fl, fr, bl, br);
    }

    /*public void initSpinner() {
        spin = hardwareMap.get(DcMotorEx.class, "spin");

        spin.setDirection(DcMotorSimple.Direction.REVERSE);

        spinner = new Spinner(spin);
    }*/

    public void initIO() {
        lift1 = hardwareMap.get(DcMotorEx.class, "leftLift");
        lift2 = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        io = new IO(lift1, lift2, leftClaw, rightClaw);
    }

    public void initStateEstimator() {
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        IMUParameters.temperatureUnit     = BNO055IMU.TempUnit.FARENHEIT;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUParameters.loggingEnabled      = true;
        IMUParameters.loggingTag          = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //NaiveAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUParameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        estimator = new StateEstimator(new IMU(imu), new MKE(fl, fr, bl, br), new OCV(webcam), scanning);
    }

    public double getDt() {
        return dt;
    }

    public Drive getDrive() {
        return drive;
    }

    public IO getIo() {
        return io;
    }

    public StateEstimator getEstimator() {
        return estimator;
    }

    public void stopRobot(){
        for (Subsystem system: subsystems) {
            system.stop();
        }
    }

    public void setScanning(boolean scanning) {
        this.scanning = scanning;
    }
}
