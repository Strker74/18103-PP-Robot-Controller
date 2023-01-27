package org.firstinspires.ftc.teamcode.dreamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dreamcode.Robot;
import org.firstinspires.ftc.teamcode.dreamcode.States.DriveMode;

@TeleOp
public class TeleOpTesting extends Robot {
    int oc = 1;
    //String adjust = "kpu";

    @Override
    public void init() {
        super.init();
        super.getIo().openClaw();
    }

    @Override
    public void loop() {
        super.loop();
        super.getDrive().POVMecanumDrive(-gamepad1.left_stick_y,
                gamepad1.left_stick_x, gamepad1.right_stick_x, DriveMode.Optimized);

        if (gamepad1.a) {
            switch (oc) {
                case 1:
                    super.getIo().closeClaw();
                    oc = 0;
                    break;
                case 0:
                    super.getIo().openClaw();
                    oc = 1;
                    break;
            }
        }
        if (gamepad1.b) {
            super.getIo().changeAdjust();
        }
        if(gamepad1.y){
            switch(super.getIo().adjust){
                case "kpu": super.getIo().pidKpIncAdjust(); break;
                case "ki" : super.getIo().pidKiIncAdjust(); break;
                case "kd" : super.getIo().pidKdIncAdjust(); break;
            }
        }
        if(gamepad1.x){
            switch(super.getIo().adjust){
                case "kpu": super.getIo().pidKpDecAdjust(); break;
                case "ki" : super.getIo().pidKiDecAdjust(); break;
                case "kd" : super.getIo().pidKdDecAdjust(); break;
            }
        }
        if (gamepad1.dpad_up) {super.getIo().setLiftHigh();}
        if (gamepad1.dpad_left || gamepad1.dpad_right) {super.getIo().setLiftMid();}
        if (gamepad1.dpad_down) {super.getIo().setLiftLow();}
        if (gamepad1.left_bumper) {super.getIo().setLiftDown(); super.getIo().resetLift();}
        if (gamepad1.right_bumper) {super.getIo().changeScalar();}
        super.getIo().PosAdjustLift(gamepad1.right_trigger - gamepad1.left_trigger);


    }
}
