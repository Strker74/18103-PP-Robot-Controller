package org.firstinspires.ftc.teamcode.dreamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@TeleOp
public class Main extends Robot {
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        super.getDrive().POVMecanumDrive(-gamepad1.left_stick_y,
                gamepad1.left_stick_x, gamepad1.right_stick_x, DriveMode.Optimized);

        super.getIo().runLift(gamepad1.right_stick_y);

        if (gamepad1.a) {super.getIo().openClaw();}
        if (gamepad1.b) {super.getIo().closeClaw();}

    }
}
